//! Program run state machine (idle / running / paused). Transitions return whether
//! they happened; the controller performs the side effects (emit, rewind, hold…).

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum WorkflowState {
    #[default]
    Idle,
    Running,
    Paused,
}

impl WorkflowState {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Idle => "idle",
            Self::Running => "running",
            Self::Paused => "paused",
        }
    }
}

#[derive(Debug, Default)]
pub struct Workflow {
    pub state: WorkflowState,
}

impl Workflow {
    pub fn is_idle(&self) -> bool {
        self.state == WorkflowState::Idle
    }

    pub fn is_running(&self) -> bool {
        self.state == WorkflowState::Running
    }

    pub fn is_paused(&self) -> bool {
        self.state == WorkflowState::Paused
    }

    pub fn start(&mut self) -> bool {
        self.transition(self.state != WorkflowState::Running, WorkflowState::Running)
    }

    pub fn stop(&mut self) -> bool {
        self.transition(self.state != WorkflowState::Idle, WorkflowState::Idle)
    }

    pub fn pause(&mut self) -> bool {
        self.transition(self.state == WorkflowState::Running, WorkflowState::Paused)
    }

    pub fn resume(&mut self) -> bool {
        self.transition(self.state == WorkflowState::Paused, WorkflowState::Running)
    }

    fn transition(&mut self, allowed: bool, to: WorkflowState) -> bool {
        if allowed {
            self.state = to;
        }
        allowed
    }
}
