import pandas as pd
import numpy as np



def read_csv(file_name: str):
    condition, measure_distance = file_name.split(".")[0].split("_")[2:]
    print(condition)
    print(measure_distance)
    df = pd.read_csv(file_name)
    print(df.head)



file_name = "csvs/sample_data_dark_5cm.csv"
read_csv(file_name)