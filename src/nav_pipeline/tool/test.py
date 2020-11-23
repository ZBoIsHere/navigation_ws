import os
from pathlib import Path

print(os.getcwd())
print(Path(__file__).parent.absolute())

data_dir = str(Path(__file__).parent.absolute()) + "/../data"
print(data_dir)
data_dir = data_dir + "/%d.json"
print(data_dir)