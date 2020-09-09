import os
keyword = input("Enter a keyword belonging to the file(s) you wish to delete: ")
directory = "./test/simDefinitions"

for fileName in os.listdir(directory):
    if keyword in fileName:
        os.remove(directory + "/" + fileName)