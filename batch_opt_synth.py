import argparse
import subprocess
import os
import filecmp

def main():
    # Argument Parser
    parser = argparse.ArgumentParser(description='Perform batch optimizations using bayesopt_synth')
    parser.add_argument(      "--binary"       , default="bayesopt_synth", help="Binary Optimization filepath"           )
    parser.add_argument("-n", "--n_tests"      , default=1     , type=int, help="Number of tests to perform (default: 1)")
    parser.add_argument("-i", "--initial_index", default=0     , type=int, help="Initial test index         (default: 0)")

    args = parser.parse_args()

    assert args.n_tests > 0
    assert args.initial_index >= 0

    if os.path.exists(args.binary):
        binary_path = os.path.normpath(args.binary)
    else:
        print "Binary doesnt exist! No batch tests can be performed"
        print args.binary
        exit(-1)

    for idx in range(args.initial_index, args.initial_index+args.n_tests):
        print "Batch test: " + str(idx) + " / " + str(args.initial_index+args.n_tests-1)

        if str(binary_path)[0] != '/':
            subprocess_args = ["./" + str(binary_path), "-i", str(idx), "-n", str(args.initial_index+args.n_tests)]
        else:
            subprocess_args = [       str(binary_path), "-i", str(idx), "-n", str(args.initial_index+args.n_tests)]

        print subprocess_args

        subprocess.call(subprocess_args)


if __name__ == "__main__":
    main()
