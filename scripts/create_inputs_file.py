#!/usr/bin/env python

import os
import argparse

def main():
    parser = argparse.ArgumentParser('Get megapose results')
    parser.add_argument('--name', type=str, help="name of the example dir")
    parser.add_argument("x1", type=int)
    parser.add_argument("y1", type=int)
    parser.add_argument("x2", type=int)
    parser.add_argument("y2", type=int)

    args = parser.parse_args()

    print(args.x2)

if __name__ == '__main__':
    main()