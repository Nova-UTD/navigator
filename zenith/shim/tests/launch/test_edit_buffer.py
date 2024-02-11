from launch.edit_buffer import EditBuffer

def test_init():
    EditBuffer("tests/data/edit_buffer_file.txt")

def test_write():
    eb = EditBuffer("")
    eb.write("test")
    assert eb.position == 4
    assert eb.contents == "test"

def test_delete_until():
    eb = EditBuffer("test")
    eb.set_position(1)
    eb.delete_until("t")
    assert eb.contents == "tt"

def test_delete_until_multiline():
    eb = EditBuffer("test\nline")
    eb.delete_until("l")
    assert eb.contents == "line"

def test_skip_lines():
    eb = EditBuffer("test\nline\nskip\nme")
    eb.skip_lines(2)
    assert eb.position == 10

def test_skip_line():
    eb = EditBuffer("test\nline\nskip\nme")
    eb.skip_line()
    assert eb.position == 5

def test_jump_to_start():
    eb = EditBuffer("test")
    eb.set_position(2)
    eb.jump_to_start()
    assert eb.position == 0

def test_jump_to_relative():
    eb = EditBuffer("test\nline\nskip\nme")
    eb.jump_to_relative("line")
    assert eb.position == 5
    eb.jump_to_relative("test")
    assert eb.position == 5
    eb.jump_to_relative("skip")
    assert eb.position == 10

def test_eof():
    eb = EditBuffer("test")
    eb.skip_line()
    assert eb.eof() == True

def test_new_line():
    eb = EditBuffer("test")
    eb.new_line("line")
    assert eb.contents == "line\ntest"
    assert eb.position == 5
    eb.new_line("another")
    assert eb.contents == "line\nanother\ntest"
    assert eb.position == 13
