import glob, os, shutil


def main():
    for file in glob.glob("*.ply"):
        n= file.split(".")[0][-2::]
        shutil.move(file, "tless"+n)
        print(n)

if __name__ == '__main__':
    main()