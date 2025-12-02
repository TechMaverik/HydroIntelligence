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

users = ref.get()
print(users)
