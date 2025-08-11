import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.utils import uri_helper
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

_uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
_cf = None
_scf = None

def init_cf():
    global _cf, _scf
    if _cf is not None:
        return _cf

    print("Initializing Crazyflie...")
    cflib.crtp.init_drivers()
    _scf = SyncCrazyflie(_uri, cf=Crazyflie(rw_cache='./cache'))
    _scf.__enter__()
    _cf = _scf.cf

    _cf.param.set_value('stabilizer.estimator', '2')
    _cf.param.set_value('locSrv.extQuatStdDev', 8.0e-3)
    _cf.param.set_value('locSrv.extPosStdDev', 0.001)
    # _cf.param.set_value('locSrv.flowStdDev', 10.0)      # large covariance for flow deck (optical flow)

    return _cf

def get_scf():
    if _scf is None:
        raise RuntimeError("Crazyflie not initialized. Call init_cf() first.")
    return _scf

def get_cf():
    if _cf is None:
        raise RuntimeError("Crazyflie not initialized. Call init_cf() first.")
    return _cf

def shutdown_cf():
    global _cf, _scf
    if _cf is not None:
        print("Shutting down Crazyflie...")
        _cf.close_link()
    if _scf is not None:
        _scf.__exit__(None, None, None)
