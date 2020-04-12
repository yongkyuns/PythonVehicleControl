from pyqtgraph.Qt import QtCore, QtGui
import traceback
import sys
from multiprocessing import Process, Pool, Queue, Pipe
import time


class WorkerSignals(QtCore.QObject):
    '''
    Defines the signals available from a running worker thread.

    Supported signals are:

    finished
        No data
    
    error
        `tuple` (exctype, value, traceback.format_exc() )
    
    result
        `object` data returned from processing, anything

    progress
        `int` indicating % progress 

    '''

    finished = QtCore.pyqtSignal()
    error = QtCore.pyqtSignal(tuple)
    result = QtCore.pyqtSignal(object)
    progress = QtCore.pyqtSignal(int)


class Worker(QtCore.QRunnable):
    '''
    Worker thread

    Inherits from QRunnable to handler worker thread setup, signals and wrap-up.

    :param callback: The function callback to run on this worker thread. Supplied args and 
                     kwargs will be passed through to the runner.
    :type callback: function
    :param args: Arguments to pass to the callback function
    :param kwargs: Keywords to pass to the callback function

    '''

    def __init__(self, fn, *args, progress_fn=None, useMultiProcessing=False, verbose=False, **kwargs):
        super().__init__()

        # Store constructor arguments (re-used for processing)
        self.fn = fn
        self.useMultiProcessing = useMultiProcessing
        self.args = args
        self.kwargs = kwargs
        self.signals = WorkerSignals()   
        self.verbose = verbose 
        self.progress_fn = progress_fn

        process_pipe, main_pipe = Pipe()
        self.receiver = main_pipe
        self.sender = process_pipe

        # Add the callback to our kwargs
        # self.kwargs['progress_callback'] = self.signals.progress     

    def progress_update(self, queue, progress):   
        # queue.put(progress)
        # get_progress(queue)
        # self.signals.progress.emit(progress)
        if self.progress_fn is not None:
            self.progress_fn(progress)

    @QtCore.pyqtSlot()
    def run(self):
        '''
        Initialise the runner function with passed args, kwargs.
        '''
        time_begin = time.time()
        if self.verbose:
            print('Starting process...')

        # Retrieve args/kwargs here; and fire processing using them
        try:
            if self.useMultiProcessing is False:
                # Only use threading, without using multiprocessing.Process
                result = self.fn(*self.args, **self.kwargs)
            else:
                # Process using multiprocessing.Pool
                # This interface is consistent with threading, but progress cannot be communicated.
                # Hence below approach using multiprocessing.Process and Queue is used.
                # p = Pool(processes=1)  
                # results = p.map(self.fn, [*self.args])
                # result = results[0]
                # p.close()

                # The function input has 2 additional arguements (Queue and callback function).
                # This interface is not straightforward and needs to be fixed later.
                queue = Queue()
                p = Process(target=self.fn, args=[*self.args, queue, self.sender], kwargs=self.kwargs, daemon=True)
                p.start()

                while queue.empty():
                    try:
                        # progress = self.receiver.poll(timeout=None)
                        progress = None
                        while self.receiver.poll():
                            progress = self.receiver.recv()
                    except EOFError:
                        break
                    else:
                        self.signals.progress.emit(progress)
                result = queue.get()
                p.join()

        except:
            traceback.print_exc()
            exctype, value = sys.exc_info()[:2]
            self.signals.error.emit((exctype, value, traceback.format_exc()))
        else:
            self.signals.result.emit(result)  # Return the result of the processing
            if self.verbose:
                print('Process finished!! Time elapsed = ' + ('%.3f' %(time.time()-time_begin)) + ' sec')
        finally:
            self.signals.finished.emit()  # Done


# class SimpleThread(QtCore.QThread):
#     finished = QtCore.pyqtSignal(object)

#     def __init__(self, queue, callback, parent=None):
#         QtCore.QThread.__init__(self, parent)      
#         self.queue = queue
#         self.finished.connect(callback)

#     def run(self):
#         while True:
#             arg = self.queue.get() 
#             if arg is None: # None means exit
#                 print("Shutting down")
#                 return
#             self.fun(arg)  

#     def fun(self, sim):
#         print('Running ' + str(sim.sim_time))
#         log = sim.run()
#         print('Finished ' + str(sim.sim_time))
#         self.finished.emit(log)