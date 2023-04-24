import numpy as np
import argparse
import matplotlib.pyplot as plt
import pickle



def __main__():
 parser = argparse.ArgumentParser(
                    prog='histogram displayer',
                    description='displays histogram from file',
                    epilog='Show some histogram :)')
 parser.add_argument('-f', '--filename', help="path to File with data", required=True)
 parser.add_argument('-b', '--bins', help="number of bins", required=False)
 args = parser.parse_args()
 if args.filename==None:
    print('required path to file\n')
    return -1
 else:
        b=0
        if args.bins==None:
            b=20
        else:
            b=int(args.bins)

        print(args.filename)

        with open(str(args.filename), 'rb') as f:
            data = pickle.load(f)
        plt.hist(np.array(data), bins=b)
        plt.show()
if __name__ == '__main__':
    a=1
    try:
        __main__()
    except __main__:
        pass
