# Draw text to center of screen
import curses

screen = curses.initscr()
num_rows, num_cols = screen.getmaxyx()

# Make a function to print a line in the center of screen
def print_center(message):
    # Calculate center row
    middle_row = int(num_rows / 2)

    # Calculate center column, and then adjust starting position based
    # on the length of the message
    half_length_of_message = int(len(message) / 2)
    middle_column = int(num_cols / 2)
    x_position = middle_column - half_length_of_message

    # Draw the text
    screen.addstr(middle_row, x_position, message)
    screen.refresh()


print_center("Hello from the center!")

# Wait and cleanup
curses.napms(3000)
curses.endwin()