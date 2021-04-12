import joblib
from joblib import Parallel, delayed
from collections import defaultdict


def unwrap_self(arg, **kwarg):
    return arg[0].run_one_test_parallel(*(arg[1:]), **kwarg)


class TestRunner:
    """
    Class in order to conduct tests on the RANSAC for 3D lines
    """

    def __init__(self, parameters):
        """

        :param parameters: an array of parameters to give to run_one_test_parallel for each test.
        If the function run_one_test_parallel has no argument, give [None]*number_of_tests
        Otherwise, each element of parameters is a dict whose (key, value) pairs are the parameters of the function
        run_one_test_parallel.
        """
        self.accuracy_std_dev_results = None
        self.parameters = parameters
        self.results = []

    def make_test_monitor(self):
        """
        Function in order to monitor the progress of the tests. It re-implements the BatchCompletionCallBack
        of joblib in order to adapt it to the number of tests we have.
        :return:
        """
        total_number_of_param_sets = len(self.parameters)

        class BatchCompletionCallBack(object):
            completed = defaultdict(int)

            def __init__(self, time, index, parallel):
                self.index = index
                self.parallel = parallel

            def __call__(self, index):
                BatchCompletionCallBack.completed[self.parallel] += 1
                print("Progress : %s %% " %
                      str(BatchCompletionCallBack.completed[self.parallel] * 100 / total_number_of_param_sets))
                if self.parallel._original_iterator is not None:
                    self.parallel.dispatch_next()

        joblib.parallel.BatchCompletionCallBack = BatchCompletionCallBack

    def run_one_test_parallel(self, parameters):
        """

        :param parameters: a dictionary with the parameters
        :return: Must return an array containing the measurable values
        """
        pass

    def run_all_tests_parallel(self, parallel=True):
        """
        Run all the tests in a parallel way
        :return:
        """
        # Prepare the monitoring system
        self.make_test_monitor()
        # Run the tests
        if parallel:
            self.results = Parallel(n_jobs=-1, backend="multiprocessing") \
                (delayed(unwrap_self)(i) for i in zip([self] * len(self.parameters), self.parameters))
        else:
            self.results = [unwrap_self(r) for r in zip([self] * len(self.parameters), self.parameters)]
        return self.results
