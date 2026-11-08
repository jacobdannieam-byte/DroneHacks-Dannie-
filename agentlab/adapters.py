"""Fail-closed activation boundaries. No external process or network execution."""
class DisabledAdapter:
    def execute(self,*args,**kwargs):
        raise PermissionError('Not activated or live qualified; execution unavailable in this build')
CodexWorker = DisabledAdapter
GazeboRunner = DisabledAdapter
NotionPublisher = DisabledAdapter
PhysicalFlight = DisabledAdapter
