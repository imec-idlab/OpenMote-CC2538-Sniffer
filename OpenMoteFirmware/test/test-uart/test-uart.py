#!/usr/bin/python

'''
@file       test-uart.py
@author     Pere Tuset-Peiro  (peretuset@openmote.com)
@version    v0.1
@date       May, 2015
@brief      

@copyright  Copyright 2015, OpenMote Technologies, S.L.
            This file is licensed under the GNU General Public License v2.
'''

import os, sys

library_path = os.path.abspath('../../library')
sys.path.append(lib_path)

import Serial

def program():
    print "Hello world!"

def main():
    program()
    
if __name__ == "__main__":
    main()
