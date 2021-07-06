import yaml


def get_params(params_path):
    with open(params_path, 'r') as fle:
        params = yaml.safe_load(fle)
    return params
