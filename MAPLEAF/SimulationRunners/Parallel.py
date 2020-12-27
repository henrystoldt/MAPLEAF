from .SingleSimulations import Simulation

__all__ = [ "RemoteSimulation" ]

try:
    import ray

    @ray.remote
    class RemoteSimulation(Simulation):
        ''' 
            Exactly the same as Simulation, except the class itself, and its .run method are decorated with ray.remote()
            to enable multithreaded/multi-node simulations using [ray](https://github.com/ray-project/ray)
        '''
        @ray.method(num_returns=2)
        def run(self):
            return super().run()

except (ImportError, AssertionError):
    class RemoteSimulation():
        """
            This is a fake remote simulation class which will raise an exception if it is ever used.
            If it is not used (no parallel simulations are run) then no error should be raised
        """
        def __init__(self):
            raise ImportError("Problem importing ray, make sure the version matching that requested in requirements.txt is installed and working!")