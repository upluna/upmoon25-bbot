from curses import wrapper

def main(stdscr):
    stdscr.clear()

    while True:
        char = stdscr.getch()
        print("Got ", char)

if __name__ == '__main__':
    wrapper(main)
