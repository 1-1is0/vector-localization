from re import A
import pandas as pd
import numpy as np
from pathlib import Path
from itertools import islice
import os

def read_file(file_name: str):
    df = pd.DataFrame()
    with open(file_name, 'r') as txt_file:
        data = {}
        for line in txt_file.readlines():
            if "distance" in line:
                if len(data) != 0:
                    # print(data)
                    df = pd.concat([df, pd.DataFrame([data])], ignore_index=True)
                data = {}
            ent = line.strip().split(":")
            data.update({ent[0]: ent[1]})

    # print(df.head())
    df['distance'] = df['distance'].astype(float)
    df['found_object'] = df['found_object'].astype(bool)
    df['is_lift_in_fov'] = df['is_lift_in_fov'].astype(bool)
    df['unobstructed'] = df['unobstructed'].astype(bool)
    print(df.info())
    # print(df.loc[df['signal_quality'].str.strip()=='6.437743186950684'])
    df.to_csv(file_name.replace('txt', 'csv'), index=False)




from glob import glob
def main():

    all_txt_file = glob("txts/*.txt")
    os.makedirs('csvs', exist_ok=True)
    print(all_txt_file)
    for txt_file in all_txt_file:
        print(txt_file)
        read_file(txt_file)

main()