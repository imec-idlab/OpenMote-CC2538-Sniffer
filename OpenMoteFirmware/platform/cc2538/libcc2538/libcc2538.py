import os, sys, subprocess

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

def rename_files(folder, extension):
    result = []
    print("Renaming files..."),
    for filename in os.listdir(folder):
        name_, extension_ = filename.split(".")
        if ((extension_ == extension)):
            processed = name_.endswith("_")
            if (not processed):
                name = name_ + "_." + extension_
                src = os.path.join(folder, filename)
                dst = os.path.join(folder, name)
                os.rename(src, dst)
            else:
                name = filename
            result.append(name)
    print("ok!")
    return result
    
def execute_makefile():
    print("Executing Makefile..."),
    subprocess.call(["make"], stdout=devnull, stderr=devnull, shell=False)
    print("ok!")
    
def copy_library():
    print("Copying libcc2538..."),
    subprocess.call(["mv", "libcc2538.a", "../libcc2538.a"], shell=False)
    print("ok!")
    
def revert_files():
    print("Reverting source files..."),
    subprocess.call(["rm", "-rf", "bin"], shell=False)
    subprocess.call(["rm", "-rf", "src"], shell=False)
    subprocess.call(["git", "checkout", "src"], shell=False)
    print("ok!")

def main():
    src_files = rename_files(src_folder, src_extension)
    if (src_files):
        generate_makefile(src_folder, src_files)
        execute_makefile()
        copy_library()
        revert_files()
    else:
        print("error!")
        
if __name__ == "__main__":
    main()

