import os
import argparse

class dir_crawler:
    #root_dir = ""       # Where to start the traversal
    #out_dir = ""        # Where to print the output file to
    #out_filename = ""   # Name of file to print to
    #file_type = ""      # Look for these kinds of files
    #pwd = ""            # PWD when script is launched
    #file_paths = []     # Output list for all targeted bag files


    def __init__(self, root_dir, out_dir, out_filename, file_type, verbose):
        self.root_dir = root_dir
        self.out_dir = out_dir
        self.out_filename = out_filename
        self.file_type = file_type
        self.pwd = ""
        self.verbose = verbose
        self.file_paths = []

    def traverse(self):
        stash_cwd = os.getcwd()

        queue = []
        queue.append(os.path.abspath(self.root_dir))
        for item in queue:
            if self.verbose:
                print(item)
            os.chdir(item)
            for name in os.listdir(item):
                if os.path.isdir(name):
                    queue.append(os.path.abspath(name))
                elif os.path.isfile(name):
                    text,ext = os.path.splitext(name)
                    if ext == self.file_type or self.file_type == "all":    #TODO: maybe regex to make this cleaner
                        self.file_paths.append(os.path.abspath(name))

        if self.verbose:
            print("Found {} files".format(len(self.file_paths)))
        os.chdir(stash_cwd)

    def reorganize(self):
        self.file_paths.sort()

    def get_dirs(self):
        dir_list = []
        for file in self.file_paths:
            this_dir = os.path.dirname(file)
            if this_dir not in dir_list:
                dir_list.append(this_dir)
        return dir_list

    def writeout(self):
        #os.chdir(self.out_dir)
        with open(self.out_dir + '/' + self.out_filename, 'w') as f:
            for file_path in self.file_paths:
                f.write(f"{file_path}\n")

    def printout(self):
        return self.file_paths
        

if __name__ == "__main__":

    default_inpath = "/media/storage/wriva_deliverables/231016-siteA01-delivery/231016-siteA01-delivery/siteA01-SIT-campus/camA003-gopro-hero-mini-11" 
    default_outfile = "configuration/crawler_output.txt"

    def handle_args(args):
        if not args.inpath:
            args.inpath = default_inpath
        if not args.outfile:
            args.outfile = default_outfile

        if not os.path.exists(args.inpath):
            print("Error: file does not exist")
            return False

        return True

    parser = argparse.ArgumentParser(
        prog="Directory Crawler",
        description="Recursively find paths to files of specified type"
    )

    parser.add_argument('-f',
                        '--filetype',
                        required=True,
                        type=str,
                        metavar="",
                        help="Type of file to search for")

    parser.add_argument('-i',
                        '--inpath',
                        required=False,
                        type=str,
                        metavar="",
                        help="Top directory to start from")
    
    parser.add_argument('-o',
                        '--outfile',
                        required=False,
                        type=str,
                        metavar="",
                        help="Print list of paths to this file with this name")

    parser.add_argument('-v',
                        '--verbose',
                        help="Print dirnames during traversal",
                        action='store_true')

    args = parser.parse_args()
    if not handle_args(args):
        print("Bad argument parse: {}\n{}\n{}\n{}\nAborting".format(
            args.filetype,
            args.inpath,
            args.outfile,
            args.verbose
        ))

    else:
        out_dir = os.getcwd()
        session = dir_crawler(args.inpath, 
                            out_dir, 
                            args.outfile, 
                            args.filetype, 
                            args.verbose)
        session.traverse()
        session.reorganize()
        session.writeout()
        