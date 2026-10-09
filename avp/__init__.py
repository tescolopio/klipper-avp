"""AVP package and Klipper extras entry point."""


def load_config(config):
    from .klipper.adapter import load_config as load_adapter
    return load_adapter(config)
