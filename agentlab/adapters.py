"""Production transports are gated; default construction remains disabled."""
from .integrations import CodexWorker, GazeboRunner, NotionPublisher
class PhysicalFlight:
    def execute(self,*args,**kwargs):
        raise PermissionError('Physical flight is not supported by the lab')
