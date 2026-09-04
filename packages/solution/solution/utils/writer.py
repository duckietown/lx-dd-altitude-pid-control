import yaml


def update_gains(kp, kd, ki, k, filepath='z_pid.yaml'):
    """Write PID gains to a YAML file."""
    with open(filepath, 'w') as f:
        gains = {'Kp': kp, 'Kd': kd, 'Ki': ki, 'K': k}
        yaml.dump(gains, f)


def load_gains(filepath='z_pid.yaml'):
    """Load PID gains from a YAML file.

    Returns a dict with keys: ``Kp``, ``Ki``, ``Kd``, ``K``.
    """
    with open(filepath, 'r') as f:
        gains = yaml.full_load(f)
    return gains
