from pathlib import Path

MAIN_DIR = Path(__file__).parent
DATABASE_PATH = MAIN_DIR / "resources" / "database"

# Default final approach point height above the runway
DEFAULT_FAP_HEIGHT = 2000
# Default glide slope of final approach
DEFAULT_SLOPE = 3.0

# Global variables: bottom and top height over the field of the TRACON
BOTTOM = 2000
TOP = 8000