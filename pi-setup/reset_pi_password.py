"""Set a new Pi password and trust this PC's SSH key, by editing the SD card's
cloud-init files. The password is hashed locally; only the hash is written.

Run with the Pi's SD card in this PC:  python reset_pi_password.py
"""
import getpass
import os
import re
import shutil
import subprocess
import sys

OPENSSL = r"C:\Program Files\Git\mingw64\bin\openssl.exe"
PUBKEY = os.path.expandvars(r"%USERPROFILE%\.ssh\id_ed25519.pub")
USER = "arnavk"


def find_boot():
    for letter in "DEFGHIJKLMNOPQRSTUVWXYZ":
        root = f"{letter}:/"
        if all(os.path.exists(root + f) for f in ("user-data", "meta-data", "cmdline.txt")):
            return root
    sys.exit("Couldn't find the Pi's SD card (bootfs). Is it plugged in?")


def main():
    boot = find_boot()
    print(f"Found the Pi's SD card at {boot[0]}:")

    while True:
        pw = getpass.getpass("New Pi password (nothing shows while typing): ")
        if len(pw) < 6:
            print("Use at least 6 characters.")
            continue
        if pw != getpass.getpass("Type it again: "):
            print("They didn't match. Try again.")
            continue
        break
    pw_hash = subprocess.run([OPENSSL, "passwd", "-6", "-stdin"], input=pw,
                             capture_output=True, text=True, check=True).stdout.strip()
    pubkey = open(PUBKEY, encoding="utf-8").read().strip()

    backup = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sd-backup")
    os.makedirs(backup, exist_ok=True)
    for f in ("user-data", "meta-data", "cmdline.txt"):
        shutil.copy2(boot + f, os.path.join(backup, f))

    ud = open(boot + "user-data", encoding="utf-8", newline="").read()
    ud = re.split(r"\n# --- added by reset_pi_password.py ---\n", ud)[0].rstrip("\n") + "\n"
    ud += ("\n# --- added by reset_pi_password.py ---\n"
           "ssh_authorized_keys:\n"
           f"  - {pubkey}\n"
           "chpasswd:\n"
           "  expire: false\n"
           "  users:\n"
           f"    - name: {USER}\n"
           f"      password: '{pw_hash}'\n"
           "      type: hash\n")
    open(boot + "user-data", "w", encoding="utf-8", newline="").write(ud)

    # New instance id so cloud-init runs again on next boot.
    for f in ("cmdline.txt", "meta-data"):
        t = open(boot + f, encoding="utf-8", newline="").read()
        t2, n = re.subn(r"(rpi-imager-\d+)(-[\w]+)?", r"\1-pwreset", t, count=1)
        if n != 1:
            sys.exit(f"Couldn't update {f}; nothing else is broken, tell Claude.")
        open(boot + f, "w", encoding="utf-8", newline="").write(t2)

    print("\nDone. Eject the card (right-click bootfs > Eject), put it in the Pi, power it on.")


if __name__ == "__main__":
    main()
