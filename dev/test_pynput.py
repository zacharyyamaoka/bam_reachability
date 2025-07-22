from pynput import keyboard

def run():
    print("Viewer running... Press keys to interact.")
    print("  [Space]    → Random frame")
    print("  [→ / ↑]    → Next orientation")
    print("  [← / ↓]    → Previous orientation")
    print("  [+ / =]    → Increase point size")
    print("  [- / _]    → Decrease point size")
    print("  [Q]        → Quit")

    def on_press(key):
        try:
            k = key.char  # Printable character
        except AttributeError:
            k = key.name  # Special key like 'space', 'left', etc.

        print("Key pressed: ", k)

        if k == ' ':
            print("[Space] → Random frame")
        elif k in ['right', 'up']:
            print("[→ / ↑] → Next orientation")
        elif k in ['left', 'down']:
            print("[← / ↓] → Previous orientation")
        elif k in ['+', '=']:
            print("[+] → Increase point size")
        elif k in ['-', '_']:
            print("[-] → Decrease point size")
        elif k in ['q', 'Q', 'esc'    ]:
            print("Exiting viewer...")
            return False  # Stop listener

    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()

if __name__ == "__main__":
    run()
