"""argo_v2 uses its own physics asset with the unchanged v1 motor/sensor backend."""
from .argo_v1 import ArgoV1


class ArgoV2(ArgoV1):
    def __init__(self, *args, **kwargs):
        kwargs['vehicle'] = 'argo_v2'
        super().__init__(*args, **kwargs)
