class SimulationArgs():
    num_repetitions: int = 1
    display: bool = False
    display_com: bool = False
    debug: bool = False
    debug_step: int = 1
    display_collision_model: bool = True
    display_step: bool = False
    display_state: bool = False
    display_contacts: bool = True
    debug_transparency: float = 0.5
    max_fps: int = 30
    Kp: float = 0 # baumgarte proportional term
    Kd: float = 0 # baumgarte derivative term
    compliance: float = 0.0
    material: str = "metal"
    horizon: int = 1000
    dt: float = 1e-3
    tol: float = 1e-6
    tol_rel: float = 1e-6
    mu_prox: float = 1e-4
    maxit: int = 100
    warm_start: int = 1
    contact_solver: str = "PGS"
    plot_metrics: bool = False
    plot_hist: bool = False
    plot_title: str = "NO TITLE"
    seed: int = 1234
    random_initial_velocity: bool = False
    add_damping: bool = False
    damping_factor: float = 0.0
    admm_update_rule: str = "spectral"
    mujoco_show_ui: bool = False
    max_patch_size: int = 2
    patch_tolerance: float = 1e-2

    def process_args(self):
        if self.debug:
            self.display = True
            self.display_contacts = True
            self.display_state = True
            self.display_com = True

class ControlArgs(SimulationArgs):
    noise_scale: float = 5.0
    nnodes: int = 10 # nnodes = horizon // nsteps
    tau_max: float = 20.0
    wtau: float = 1e-4
    wtarget: float = 5.0
    wvel: float = 1.0
    Nsim: int = 25
    max_fevals: int = 1e4
    use_max_fevals: int = 1
