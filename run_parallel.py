from pyvehsim.core.vehicle import Vehicle
from pyvehsim.core.simulator import Simulator
from pyvehsim.helper.logger import Logger
import cProfile
import sys
# from pyvehsim.helper.qt_worker import Worker
from pyqtgraph.Qt import QtCore

from threading import Thread
import threading
import multiprocessing
from multiprocessing.pool import ThreadPool
import traceback
import logging
import concurrent.futures

sys.path.append('.')

logging.basicConfig(level=logging.DEBUG,
                    format='[%(levelname)s] (%(threadName)-10s) %(message)s',
                    )

def run(sim):
    # logging.debug('Starting')
    # results = []
    # for sim in sims:
    #     log = sim.run()
    #     results.append(log)
    
    # logging.debug('Exiting')
    return sim.run()

def finished():
    print('sim finished')

def progress():
    print('progress')


class SimProcess(multiprocessing.Process):
    def __init__(self,sim,sender_pipe):
        multiprocessing.Process.__init__(self)
        self.sender = sender_pipe
        self.sim = sim
    
    def run(self):
        result = self.sim.run(self.sender)
        self.sender.send(result)

def run_sim(sim):
    sender, receiver = multiprocessing.Pipe()
    worker = SimProcess(sim, sender)
    worker.start()

    while True:
        result = receiver.recv()
        if isinstance(result, Logger):
            break
        else:
            print(result)

    worker.join()
    return result


def main():
    logging.basicConfig(level=logging.DEBUG)
    multiprocessing.set_start_method('spawn')

    sim_time = 3.3
    verbose = False
    sim = Simulator(sim_time=sim_time, verbose=verbose, controller='PID')
    sim2 = Simulator(sim_time=sim_time, verbose=verbose, controller='DDPG')
    sim3 = Simulator(sim_time=sim_time, verbose=verbose, controller='MPC')

    sims = [sim] * 1

    # threads = []
    # t = Thread(target=run_sim,args=(sim,))
    # threads.append(t)
    # t.start()
    # t.join()
    # results = result.get()
    # for result in results:
    #     print(result.x[-1])
    results = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=8) as executor:
        for s in sims:
            r = executor.submit(run_sim,s)
            results.append(r)

    for r in results:
        log = r.result()
        print(log.x[-1])

        # run1 = executor.submit(run_sim, sim)
        # run2 = executor.submit(run_sim, sim3)
        # result1 = run1.result()
        # result2 = run2.result()
        # print(result1.x[-1])
        # print(result2.x[-1])
    # result = run_sim(sim2)
    # print(str(result.x[-1]))


    # jobs = []
    # p = multiprocessing.Process(target=run,args=(sims,))
    # jobs.append(p)
    # p.start()

    # worker = Worker(run, sim, verbose=True, useMultiProcessing=False)
    # worker.signals.result.connect(finished)
    # worker.signals.progress.connect(progress)

    # threadpool = QtCore.QThreadPool()
    # threadpool.start(worker)

    # sim.run()
    # sim2.run()

if __name__ == '__main__':
    main()
