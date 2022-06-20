import os

# https://stackoverflow.com/a/63261669
all_folders = os.listdir(os.getcwd())
all_folders.sort()
latest = all_folders[-1].replace('dataset', '')
new = int(latests) + 1
os.makedirs('ID'+str(latest))