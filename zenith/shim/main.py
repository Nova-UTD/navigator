import sys

from subsystem import SubsystemMap

def main():
    print(SubsystemMap().to_json(), file=sys.stdout)

if __name__ == '__main__':
    main()