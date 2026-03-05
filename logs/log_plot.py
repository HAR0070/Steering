import matplotlib.pyplot as plt 
import re 
import pandas as pd
import numpy as np
from pathlib import Path 

def plot_pos(data: pd.DataFrame) :

    # fig, ax = plt.subplots(figsize=(20, 6))
    x = data.iloc[int(0.7*len(data)):int(0.725 * len(data))].copy()
    x.Speed = x.Speed/10000 
    
    x['speed_integral'] = x.Speed.where(abs(x.Speed) > 0.01 , 0).cumsum()
    x['position_diff'] = x.Pos.where(abs(x.Pos) > 0.1 , 0).diff()

    cols = ["Pos",'speed_integral', 'position_diff', "Speed", "Curr", "Temp", "err"]
    for c in cols:
        fig, ax = plt.subplots(figsize=(20, 6))
        ax.plot(x.index, x[c])
        ax.set_title(c)

        # # Uniform 0.2s grid
        # ax.set_xticks(np.arange(0, x.index.max(), 0.2))

        # for i, label in enumerate(ax.get_xticklabels()):
        #     if i % 50 != 0:   # show every 1 second
        #         label.set_visible(False)

        # ax.set_xlabel("Time (s)")
        # ax.legend(c)
        # ax.grid(True)
    
    plt.tight_layout()
    plt.show()

def parser(file_name , debug = False):
    
    # file_name = "_log32.txt"  #input("Enter the file name - file should be in same director")
    # debug = False 

    path = Path(f"{file_name}")
    if not path.exists():
        path = Path(f"{file_name}.txt")
        if not path.exists():
            print(f"C man this filename is wrong {file_name} - check again if its a txt file and is present in this directory")

    if path.exists():
        try:
            with open(path , 'r') as file:
                i = 0
                columns = ['time', 'Pos' , 'Speed' , 'Curr' , 'Temp' , 'err']
                data = pd.DataFrame(columns=columns)
                
                # print(data.columns)
                
                for line in file:
                    content = line.strip().split(" ")
                    check = set(content)
                    if "RX" in check: 
                        if debug:
                            print(f" error is {content[-1]} , temp {content[-4][:-1]} , curr {content[-8]} , speed {content[-12]} , position {content[-16]}")
                        new_row = pd.DataFrame( [[content[0][1:-1], float(content[-16]) , float(content[-12]) , float(content[-8]) , float(content[-4][:-1]) , float(content[-1])]] ,
                            columns= data.columns)
                    else:
                        new_row = pd.DataFrame( [[content[0][1:-1] ,np.nan , np.nan , np.nan , np.nan , np.nan]] , columns= data.columns)
                        
                    data = pd.concat([data, new_row], ignore_index=True)
                
                data.set_index('time' , inplace=True)
                data.index = pd.to_datetime(data.index)  #strftime("%H:%M:%S.%f")
                t0 = data.index[0]
                data.index = (data.index - t0).total_seconds()
                        
        except Exception as e:
            print(f"Dude this isn't working : {e}")
            
    return data 


if '__init__' == '__main__':
    file_name = input("Enter the file name - file should be in same director  eg. _log32.txt ")
    data = parser(file_name)
    plot_pos(data)