//! Watch directory: lists, reads, writes, renames and deletes G-code files, and
//! broadcasts `watchdir:change` on file system changes.

use std::fs;
use std::io;
use std::path::{Component, Path, PathBuf};
use std::sync::{Arc, Mutex};

use notify::{RecommendedWatcher, RecursiveMode, Watcher};
use serde_json::{json, Value};
use tracing::{error, info};

use crate::hub::Hub;
use crate::util::iso8601;

#[derive(Debug)]
pub enum MonitorError {
    NotConfigured,
    InvalidName,
    NotFound,
    NotFile,
    Exists,
    /// Rename left both names in place.
    Partial(String),
    Io(io::Error),
}

impl MonitorError {
    /// HTTP status the API answers with.
    pub fn status(&self) -> u16 {
        match self {
            Self::NotConfigured | Self::InvalidName | Self::NotFile => 400,
            Self::NotFound => 404,
            Self::Exists => 409,
            Self::Partial(_) | Self::Io(_) => 500,
        }
    }
}

impl std::fmt::Display for MonitorError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NotConfigured => f.write_str("Watch directory is not configured"),
            Self::InvalidName => f.write_str("Invalid file name"),
            Self::NotFound => f.write_str("File not found"),
            Self::NotFile => f.write_str("Not a file"),
            Self::Exists => f.write_str("Target file already exists"),
            Self::Partial(m) => f.write_str(m),
            Self::Io(e) => write!(f, "{e}"),
        }
    }
}

impl From<io::Error> for MonitorError {
    fn from(e: io::Error) -> Self {
        if e.kind() == io::ErrorKind::NotFound {
            Self::NotFound
        } else {
            Self::Io(e)
        }
    }
}

#[derive(Default)]
pub struct Monitor {
    root: Option<PathBuf>,
    watcher: Mutex<Option<RecommendedWatcher>>,
}

impl std::fmt::Debug for Monitor {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Monitor").field("root", &self.root).finish()
    }
}

fn file_type(ft: fs::FileType) -> &'static str {
    use std::os::unix::fs::FileTypeExt;
    if ft.is_file() {
        "f"
    } else if ft.is_dir() {
        "d"
    } else if ft.is_block_device() {
        "b"
    } else if ft.is_char_device() {
        "c"
    } else if ft.is_symlink() {
        "l"
    } else if ft.is_fifo() {
        "p"
    } else if ft.is_socket() {
        "s"
    } else {
        ""
    }
}

impl Monitor {
    /// `root = None` leaves the watch directory unconfigured.
    pub fn new(root: Option<PathBuf>) -> Self {
        Self {
            root,
            watcher: Mutex::new(None),
        }
    }

    pub fn is_configured(&self) -> bool {
        self.root.is_some()
    }

    pub fn root(&self) -> Option<&Path> {
        self.root.as_deref()
    }

    /// Resolves a relative path inside the root; rejects absolute paths and `..`.
    fn resolve(&self, rel: &str) -> Result<PathBuf, MonitorError> {
        let root = self.root.as_ref().ok_or(MonitorError::NotConfigured)?;
        let rel = Path::new(rel);
        if rel
            .components()
            .any(|c| !matches!(c, Component::Normal(_) | Component::CurDir))
        {
            return Err(MonitorError::InvalidName);
        }
        Ok(root.join(rel))
    }

    /// Resolves a bare file name directly inside the root (rename/delete/write).
    fn resolve_name(&self, name: &str) -> Result<PathBuf, MonitorError> {
        let root = self.root.as_ref().ok_or(MonitorError::NotConfigured)?;
        let p = Path::new(name);
        match (p.components().next(), p.components().count()) {
            (Some(Component::Normal(_)), 1) => Ok(root.join(p)),
            _ => Err(MonitorError::InvalidName),
        }
    }

    /// Entries directly inside `root/search_path` (hidden files excluded), like the
    /// cncjs `getFiles`: `[{name, type, size, atime, mtime, ctime}]`.
    pub fn get_files(&self, search_path: &str) -> Vec<Value> {
        let Ok(dir) = self.resolve(search_path) else {
            return vec![];
        };
        let Ok(entries) = fs::read_dir(dir) else {
            return vec![];
        };
        let mut files: Vec<Value> = entries
            .filter_map(Result::ok)
            .filter(|e| !e.file_name().to_string_lossy().starts_with('.'))
            .filter_map(|e| {
                let meta = fs::symlink_metadata(e.path()).ok()?;
                use std::os::unix::fs::MetadataExt;
                let ctime = std::time::UNIX_EPOCH
                    + std::time::Duration::new(
                        meta.ctime().max(0) as u64,
                        meta.ctime_nsec().max(0) as u32,
                    );
                Some(json!({
                    "name": e.file_name().to_string_lossy(),
                    "type": file_type(meta.file_type()),
                    "size": meta.len(),
                    "atime": meta.accessed().ok().map(iso8601),
                    "mtime": meta.modified().ok().map(iso8601),
                    "ctime": iso8601(ctime),
                }))
            })
            .collect();
        files.sort_by(|a, b| a["name"].as_str().cmp(&b["name"].as_str()));
        files
    }

    /// Reads a file below the root (subdirectories allowed, no escaping).
    pub fn read_file(&self, file: &str) -> Result<String, MonitorError> {
        let path = self.resolve(file)?;
        Ok(fs::read_to_string(path)?)
    }

    /// Writes `data` to `basename(file)` inside the root.
    pub fn write_file(&self, file: &str, data: &[u8]) -> Result<(), MonitorError> {
        let name = Path::new(file)
            .file_name()
            .ok_or(MonitorError::InvalidName)?
            .to_string_lossy()
            .into_owned();
        let target = self.resolve_name(&name)?;
        fs::write(target, data)?;
        Ok(())
    }

    /// Temporary file + final path for a streamed upload of `basename(file)`;
    /// write the temp file, then [`Monitor::commit_upload`].
    pub fn upload_paths(&self, file: &str) -> Result<(PathBuf, PathBuf), MonitorError> {
        let name = Path::new(file)
            .file_name()
            .ok_or(MonitorError::InvalidName)?
            .to_string_lossy()
            .into_owned();
        let target = self.resolve_name(&name)?;
        let temp = self.resolve_name(&format!(
            ".{name}.{}.{}.tmp",
            std::process::id(),
            crate::util::now_ms()
        ))?;
        Ok((temp, target))
    }

    pub fn commit_upload(temp: &Path, target: &Path) -> Result<(), MonitorError> {
        if let Err(e) = fs::rename(temp, target) {
            let _ = fs::remove_file(temp);
            return Err(e.into());
        }
        Ok(())
    }

    /// A plain file (not a symlink/directory) directly inside the root.
    fn existing_file(&self, name: &str) -> Result<PathBuf, MonitorError> {
        let path = self.resolve_name(name)?;
        let meta = fs::symlink_metadata(&path)?;
        if !meta.is_file() {
            return Err(MonitorError::NotFile);
        }
        Ok(path)
    }

    /// Renames without ever overwriting: the new name is claimed first (hard link, or
    /// exclusive copy on file systems without links), then the old one is removed.
    pub fn rename_file(&self, file: &str, to: &str) -> Result<(), MonitorError> {
        let from = self.existing_file(file)?;
        let to_path = self.resolve_name(to)?;
        if to_path == from {
            return Ok(());
        }
        match fs::hard_link(&from, &to_path) {
            Ok(()) => {}
            Err(e) if e.kind() == io::ErrorKind::AlreadyExists => return Err(MonitorError::Exists),
            Err(e) if matches!(e.raw_os_error(), Some(1 | 95 | 38 | 18)) => {
                // EPERM / EOPNOTSUPP / ENOSYS / EXDEV: no hard links here (FAT, exFAT…).
                let mut src = fs::File::open(&from)?;
                let mut dst = match fs::OpenOptions::new()
                    .write(true)
                    .create_new(true)
                    .open(&to_path)
                {
                    Ok(f) => f,
                    Err(e) if e.kind() == io::ErrorKind::AlreadyExists => {
                        return Err(MonitorError::Exists)
                    }
                    Err(e) => return Err(e.into()),
                };
                io::copy(&mut src, &mut dst)?;
            }
            Err(e) => return Err(e.into()),
        }
        if let Err(unlink) = fs::remove_file(&from) {
            if fs::remove_file(&to_path).is_err() {
                return Err(MonitorError::Partial(format!(
                    "Renamed to {to} but {file} could not be removed; both names now exist"
                )));
            }
            return Err(unlink.into());
        }
        Ok(())
    }

    /// Deletes a plain file directly inside the root.
    pub fn delete_file(&self, file: &str) -> Result<(), MonitorError> {
        let path = self.existing_file(file)?;
        fs::remove_file(path)?;
        Ok(())
    }

    /// Starts watching the root recursively; changes broadcast `watchdir:change`.
    pub fn watch(&self, hub: Arc<Hub>) {
        let Some(root) = self.root.clone() else {
            return;
        };
        let root_for_events = root.clone();
        let watcher =
            notify::recommended_watcher(move |res: notify::Result<notify::Event>| match res {
                Ok(event)
                    if event.kind.is_create()
                        || event.kind.is_modify()
                        || event.kind.is_remove() =>
                {
                    for path in event.paths {
                        let file = path.strip_prefix(&root_for_events).unwrap_or(&path);
                        hub.broadcast(
                            "watchdir:change",
                            &[json!({ "file": path.to_string_lossy() })],
                        );
                        tracing::debug!("watchdir change: {}", file.display());
                    }
                }
                Ok(_) => {}
                Err(e) => error!("watch error: {e}"),
            });
        match watcher {
            Ok(mut w) => match w.watch(&root, RecursiveMode::Recursive) {
                Ok(()) => {
                    info!("Watching {:?} for file changes", root);
                    *self.watcher.lock().unwrap() = Some(w);
                }
                Err(e) => error!("Unable to watch {:?}: {e}", root),
            },
            Err(e) => error!("Unable to create a file watcher: {e}"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tmp_root(tag: &str) -> PathBuf {
        let p = std::env::temp_dir().join(format!("cncr-monitor-{tag}-{}", std::process::id()));
        let _ = fs::remove_dir_all(&p);
        fs::create_dir_all(&p).unwrap();
        p
    }

    #[test]
    fn rejects_escaping_names() {
        let m = Monitor::new(Some(tmp_root("escape")));
        assert!(matches!(
            m.read_file("../etc/passwd"),
            Err(MonitorError::InvalidName)
        ));
        assert!(matches!(
            m.read_file("/etc/passwd"),
            Err(MonitorError::InvalidName)
        ));
        assert!(matches!(
            m.delete_file("a/b.nc"),
            Err(MonitorError::InvalidName)
        ));
    }

    #[test]
    fn rename_never_overwrites() {
        let root = tmp_root("rename");
        let m = Monitor::new(Some(root.clone()));
        m.write_file("a.nc", b"A").unwrap();
        m.write_file("b.nc", b"B").unwrap();
        assert!(matches!(
            m.rename_file("a.nc", "b.nc"),
            Err(MonitorError::Exists)
        ));
        assert_eq!(m.read_file("b.nc").unwrap(), "B");
        m.rename_file("a.nc", "c.nc").unwrap();
        assert!(matches!(m.read_file("a.nc"), Err(MonitorError::NotFound)));
        assert_eq!(m.read_file("c.nc").unwrap(), "A");
        let names: Vec<_> = m
            .get_files("")
            .iter()
            .map(|f| f["name"].as_str().unwrap().to_string())
            .collect();
        assert_eq!(names, vec!["b.nc", "c.nc"]);
        let _ = fs::remove_dir_all(root);
    }
}
