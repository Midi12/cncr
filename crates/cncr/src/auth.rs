//! Access control: JWT access tokens (HS256, compatible with cncjs tokens signed with
//! the same `secret`), user validation and the private-network IP whitelist.

use std::net::IpAddr;

use cncr_core::config::ConfigStore;
use jsonwebtoken::{decode, encode, Algorithm, DecodingKey, EncodingKey, Header, Validation};
use serde::{Deserialize, Serialize};
use serde_json::Value;

#[derive(Debug, Clone, Serialize, Deserialize, Default, PartialEq)]
pub struct Claims {
    #[serde(default)]
    pub id: String,
    #[serde(default)]
    pub name: String,
    #[serde(default)]
    pub iat: u64,
    pub exp: u64,
}

pub fn generate_secret() -> String {
    use rand::Rng;
    const ALPHABET: &[u8] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789./";
    let mut rng = rand::rng();
    (0..32)
        .map(|_| ALPHABET[rng.random_range(0..ALPHABET.len())] as char)
        .collect()
}

/// Time span in seconds: `3600`, `90s`, `30m`, `12h`, `30d`, `2w`, `1y`, `500ms`.
pub fn parse_lifetime(s: &str) -> Option<u64> {
    let s = s.trim().to_ascii_lowercase();
    let split = s
        .find(|c: char| !(c.is_ascii_digit() || c == '.'))
        .unwrap_or(s.len());
    let (num, unit) = s.split_at(split);
    let n: f64 = num.parse().ok()?;
    let mult = match unit.trim() {
        "" | "s" | "sec" | "secs" | "second" | "seconds" => 1.0,
        "ms" => 0.001,
        "m" | "min" | "mins" | "minute" | "minutes" => 60.0,
        "h" | "hr" | "hrs" | "hour" | "hours" => 3600.0,
        "d" | "day" | "days" => 86_400.0,
        "w" | "week" | "weeks" => 604_800.0,
        "y" | "year" | "years" => 31_557_600.0,
        _ => return None,
    };
    Some((n * mult).max(1.0) as u64)
}

fn now_secs() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0)
}

pub fn issue_token(secret: &str, lifetime_secs: u64, id: &str, name: &str) -> String {
    let iat = now_secs();
    let claims = Claims {
        id: id.into(),
        name: name.into(),
        iat,
        exp: iat + lifetime_secs,
    };
    encode(
        &Header::new(Algorithm::HS256),
        &claims,
        &EncodingKey::from_secret(secret.as_bytes()),
    )
    .unwrap_or_default()
}

pub fn verify_token(secret: &str, token: &str) -> Option<Claims> {
    decode::<Claims>(
        token,
        &DecodingKey::from_secret(secret.as_bytes()),
        &Validation::new(Algorithm::HS256),
    )
    .ok()
    .map(|d| d.claims)
}

/// Configured users with `enabled` defaulting to true.
pub fn enabled_users(config: &ConfigStore) -> Vec<serde_json::Map<String, Value>> {
    config
        .get_array("users")
        .into_iter()
        .filter_map(|u| match u {
            Value::Object(m) => Some(m),
            _ => None,
        })
        .filter(|u| u.get("enabled").and_then(Value::as_bool) != Some(false))
        .collect()
}

/// A token is valid when no user is enabled, or when it names an enabled user.
pub fn validate_user(config: &ConfigStore, claims: &Claims) -> bool {
    let users = enabled_users(config);
    users.is_empty()
        || users.iter().any(|u| {
            u.get("id").and_then(Value::as_str) == Some(claims.id.as_str())
                && u.get("name").and_then(Value::as_str) == Some(claims.name.as_str())
        })
}

/// Loopback, private and link-local addresses are always allowed.
pub fn ip_allowed(ip: IpAddr, allow_remote_access: bool) -> bool {
    if allow_remote_access {
        return true;
    }
    match ip.to_canonical() {
        IpAddr::V4(v4) => v4.is_loopback() || v4.is_private() || v4.is_link_local(),
        IpAddr::V6(v6) => {
            let first = v6.segments()[0];
            v6.is_loopback() || (first & 0xfe00) == 0xfc00 || (first & 0xffc0) == 0xfe80
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn lifetimes() {
        assert_eq!(parse_lifetime("30d"), Some(2_592_000));
        assert_eq!(parse_lifetime("3600"), Some(3600));
        assert_eq!(parse_lifetime("12h"), Some(43_200));
        assert_eq!(parse_lifetime("soon"), None);
    }

    #[test]
    fn tokens_and_users() {
        let token = issue_token("s3cret", 60, "u1", "bob");
        let claims = verify_token("s3cret", &token).unwrap();
        assert_eq!((claims.id.as_str(), claims.name.as_str()), ("u1", "bob"));
        assert!(verify_token("other", &token).is_none());

        let open = ConfigStore::in_memory(json!({}));
        assert!(
            validate_user(&open, &Claims::default()),
            "no users: everyone"
        );
        let config = ConfigStore::in_memory(json!({"users": [
            {"id": "u1", "name": "bob"}, {"id": "u2", "name": "eve", "enabled": false}
        ]}));
        assert!(validate_user(&config, &claims));
        assert!(!validate_user(
            &config,
            &Claims {
                id: "u2".into(),
                name: "eve".into(),
                ..Default::default()
            }
        ));
    }

    #[test]
    fn ip_whitelist() {
        for ok in [
            "127.0.0.1",
            "10.1.2.3",
            "172.20.0.1",
            "192.168.1.9",
            "169.254.0.1",
            "::1",
            "fd00::1",
            "fe80::1",
            "::ffff:192.168.0.2",
        ] {
            assert!(ip_allowed(ok.parse().unwrap(), false), "{ok}");
        }
        for bad in ["8.8.8.8", "172.32.0.1", "2001:db8::1"] {
            assert!(!ip_allowed(bad.parse().unwrap(), false), "{bad}");
        }
        assert!(ip_allowed("8.8.8.8".parse().unwrap(), true));
    }
}
