import firebase_admin
from firebase_admin import credentials, db
from config.app_config import AppConfig

# Initialize the app with a credentials file and database URL
cred = credentials.Certificate("config/key.json")
firebase_admin.initialize_app(
    cred,
    {"databaseURL": AppConfig().DATABASE_URL},
)

ref = db.reference("DPS2025/HydroponicsHydroIntelligence")
ref.set(
    {
        "pod1": {
            "Temperature": 22,
            "Humidity": 30,
            "Light": 800,
            "PH": 6.5,
            "TDS": 300,
        },
        "pod2": {
            "Temperature": 24,
            "Humidity": 35,
            "Light": 750,
            "PH": 6.8,
            "TDS": 320,
        },
    }
)
