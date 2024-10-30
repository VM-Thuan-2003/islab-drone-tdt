class Printer:
    def __init__(self, name, ena_logs=False):
        self._name = name
        self._ena_logs = ena_logs

    def _print(self, msg_type, data):
        if self._ena_logs:
            msg = f"[{self._name}] [{msg_type}] {data}"
            if msg_type == "WARNING":
                print(f"\033[93m{msg}\033[0m")
            elif msg_type == "SUCCESS":
                print(f"\033[92m{msg}\033[0m")
            elif msg_type == "ERROR":
                print(f"\033[91m{msg}\033[0m")
            else:
                print(msg)

    def print_warning(self, data):
        self._print("WARNING", data)

    def print_success(self, data):
        self._print("SUCCESS", data)

    def print_error(self, data):
        self._print("ERROR", data)

    def print_normal(self, data):
        self._print("NORMAL", data)

