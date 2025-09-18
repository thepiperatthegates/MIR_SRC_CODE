
from pathlib import Path
import sys


base_path = Path(__file__).parent
next_path = base_path / ".."
print(next_path.resolve())