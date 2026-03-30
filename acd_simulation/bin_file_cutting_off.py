import argparse
import os

class Fix_bin():

# parse config 
# extract how big file is.
# 

# extract chirp 1
# extract chirp 2
# extract chirp 3

# extract all chirp info add to list

# chirpCfg 0 0 0 0 0 0 0 1
# chirpCfg 1 1 0 0 0 0 0 2
# chirpCfg 2 2 0 0 0 0 0 4
# frameCfg 0 2 16 0 200 1 0
# profileCfg 0 60 359 7 57.14 0 0 70 1 256 5209 0 0 158
    config_filename=""
    output_filename=""
    chrp_list= []
    number_chirps=0
    loops=0
    samples=0
    frame_len_ms=0
    Bytes_sample=4
    no_recievers=0
    no_transmitters=0
    debug=0
    def __init__(self,debug):
        self.debug=debug
        pass

    def read_config(self,filename):
        try:
            # print(filename)
            with open(filename, "r") as f:
                for line in f:
                    # print(line)
                    if "chirpCfg" in line:
                        temp=line.split(" ")
                        pass
                    if "frameCfg" in line:
                        temp=line.split(" ")
                        chirp_low=int(temp[1])
                        chirp_high=int(temp[2])
                        self.number_chirps=(chirp_high-chirp_low)+1
                        self.loops=int(temp[3])
                        self.frame_len_ms=int(temp[5])
                        pass
                    if "profileCfg" in line:
                        temp=line.split(" ")
                        self.samples=int(temp[10])
                        pass
                    if "channelCfg" in line:
                        temp=line.split(" ")
                        print(temp)
                        if(int(temp[1])==15):
                            self.no_recievers=4
                        elif(int(temp[1])==7):
                            self.no_recievers=3
                        elif(int(temp[1])==3):
                            self.no_recievers=2
                        else:
                            self.no_recievers=1
                        if(int(temp[2])==15):
                            self.no_transmitters=4
                        elif(int(temp[2])==7):
                            self.no_transmitters=3
                        elif(int(temp[2])==3):
                            self.no_transmitters=2
                        else:
                            self.no_transmitters=1

                        # self.no_transmitters=int(temp[2])
                        # self.no_recievers=int(temp[1])
            pass
        except Exception as e:
            print(e)
            print("cannot open config file--exiting")
            exit()


    
    def get_frame_size(self):
        if self.debug:      
            print(self.number_chirps,"chirps")
            print(self.loops,"loops")
            print(self.Bytes_sample,"bytes per sample")
            print(self.no_recievers, "recievers")
            print(self.samples,"samples")
        return self.number_chirps*self.loops*self.samples*self.Bytes_sample*self.no_recievers



    def get_number_frames(self, filename):
        try:
            file_size_bytes = os.path.getsize(filename)
        except Exception as e:
            print("cannot open bin file to truncate.--exiting")
            exit()
            return 
        frame_size=self.get_frame_size()
        print(frame_size)
        number_of_frames=int(file_size_bytes/frame_size)
        return number_of_frames
        
    def truncate_file(self, filename):
        try:
            file_size_bytes = os.path.getsize(filename)
        except Exception as e:
            print("cannot open bin file to truncate.--exiting")
            exit()
            return 
        frame_size=self.get_frame_size()
        remainder_to_be_chopped= file_size_bytes % frame_size
        file_remaining=file_size_bytes-remainder_to_be_chopped
        if(self.debug):
            print("file size before",file_size_bytes,"bytes")
            print("file chopped", remainder_to_be_chopped, "bytes")
            print("file bytes remaining", file_remaining,"bytes")
        with open(filename,"a") as f:
            f.truncate(file_remaining)
            pass
        
        file_size_bytes = os.path.getsize(filename)
        if(self.debug):
            print(file_size_bytes)
            print(file_size_bytes%frame_size)
        pass

    def get_no_transmitters(self):
        return self.no_transmitters
    def get_no_recievers(self):
        return self.no_recievers
    def get_loops(self):
        return self.loops
    def get_Bytes_per_component(self):
        return self.Bytes_sample
    def get_number_chirps(self):
        return self.number_chirps
    def get_samples(self):
        return self.samples

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
                    prog='Cut off last incomplete frame from file',
                    description='Reads in config and then goes ham on file',
                    epilog='Text at the bottom of help')
    
    parser.add_argument('config_filename')    
    parser.add_argument('bin_filename')
    args = parser.parse_args()
    fix_bin_file = Fix_bin(1)
    fix_bin_file.read_config(args.config_filename)
    num_frames=fix_bin_file.get_number_frames(args.config_filename)
    print(num_frames)
    # fix_bin_file.truncate_file(args.bin_filename)
