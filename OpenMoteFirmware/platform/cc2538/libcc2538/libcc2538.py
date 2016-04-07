import os, sys, subprocess, shutil

devnull = open(os.devnull, 'w')

src_folder    = "src"
src_extension = "c"

def generate_makefile(folder, filenames):
    print("Generating Makefile..."),
    makefile = os.path.join(folder, "Makefile.include")
    of = open(makefile, 'w');
    of.write("SRC_FILES += ")
    for filename in filenames:
        of.write(filename + " ")
    of.flush()
    of.close()
    print("ok")

def find_src_files(folder, extension):
    result = []
    print("Searching files..."),
    for filename in os.listdir(folder):
        name_, extension_ = filename.split(".")
        if ((extension_ == extension)):
            result.append(filename)
    print("ok!")
    return result
    
def execute_makefile():
    print("Executing Makefile..."),
    subprocess.call(["make"], shell=False)
    print("ok!")
    
def copy_library():
    print("Moving libcc2538..."),
    shutil.move("libcc2538.a", "../libcc2538.a")
    print("ok!")
    
def revert_files():
    print("Reverting source files..."),
    shutil.rmtree("bin")
    print("ok!")

def main():
    src_files = find_src_files(src_folder, src_extension)
    if (src_files):
        generate_makefile(src_folder, src_files)
        execute_makefile()
        copy_library()
        revert_files()
    else:
        print("error!")
        
if __name__ == "__main__":
    main()

