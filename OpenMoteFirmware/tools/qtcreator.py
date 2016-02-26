#!/usr/bin/python

import os
import subprocess

from distutils.spawn import find_executable

current_dir  = "."
home_dir     = ".."

ignore_folders = ['.git', 'bin', 'python', 'tools']

src_extensions = ['.c', '.cpp', '.h', '.hpp']
hdr_extensions = ['.h', '.hpp']

qtcreator_bin      = "qtcreator"
qtcreator_project  = "tools/qtcreator"
qtcreator_creator  = "QtCreator.creator"
qtcreator_files    = "QtCreator.files"
qtcreator_includes = "QtCreator.includes"
qtcreator_default  = ["-noload", "Welcome", "-noload", "QmlDesigner", "-noload", "QmlProfiler"]

# Determines valid folders
def is_valid_folder(path):
    if path not in ignore_folders:
        return True
    return False

# Returns all folders in the current path
def get_all_folders(path):
    result = []
    folders = os.walk('.').next()[1]
    for f in folders:
        if is_valid_folder(f):
            f = os.path.abspath(f)
            result.append(f)
    return result

# Determines if it is a valid source file
def is_valid_source_file(path):
    name, extension = os.path.splitext(path)
    if extension in src_extensions:
        return True
    return False
    
# Determines if it is a valid header directory
def is_valid_header_dir(path):
    name, extension = os.path.splitext(path)
    if extension in hdr_extensions:
        return True
    return False

# Gets all files from a path
def get_all_files(path):
    result = []
    for path, subdirs, files in os.walk(path):
        for f in files:
            if is_valid_source_file(f):
                r = os.path.join(path, f)
                r = os.path.abspath(r)
                result.append(r)
    return result

# Gets source files from a path
def get_src_files(path):
    src_files = []
    
    dirs = get_all_folders(path)

    for d in dirs:
        files = get_all_files(d)
        for f in files:
            if (f):
                src_files.append(f)
    src_files.sort()
    return src_files

# Gets includes folders from a path
def get_inc_dirs(path):
    inc_dirs = []
    
    dirs = get_all_folders(path)

    for d in dirs:
        files = get_all_files(d)
        for f in files:
            if (f):
                d, f = os.path.split(f)
                if d not in inc_dirs:
                    inc_dirs.append(d)
    inc_dirs.sort()
    return inc_dirs

# Writes output to file
def write_qtcreator(file_name, inc_dirs):
    fn = open(file_name, 'w')
    for inc_dir in inc_dirs:
        fn.write(inc_dir + "\n")
    fn.close()

# Prepares the QtCreator files
def qtcreator_prepare():
    print("Preparing QtCreator..."),
    
    inc_dirs  = get_inc_dirs(current_dir)
    src_files = get_src_files(current_dir)
    
    path = os.path.join(qtcreator_project, qtcreator_files)
    write_qtcreator(path, src_files)    
    
    path = os.path.join(qtcreator_project, qtcreator_includes)
    write_qtcreator(path, inc_dirs)

    print("ok!")

# Runs QtCreator
def qtcreator_run():
    qt = [qtcreator_bin, qtcreator_project] + qtcreator_default
    
    if (find_executable(qtcreator_bin)):
        print("Running QtCreator..."),
        proc = subprocess.Popen(qt, shell=False, stdin=None, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        print("ok!")
    else:
        print("Error: QtCreator not found!")

# Go to the home project directory
def change_dir():
    current_dir = os.path.realpath(__file__)
    d, f = os.path.split(current_dir)
    home = os.path.join(d, home_dir)
    os.chdir(home)

# Main function
def main():
    change_dir()
    qtcreator_prepare()
    qtcreator_run()

if __name__ == "__main__":
    main()
