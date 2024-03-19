from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from routers import subsystems, launches

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(subsystems.router)
app.include_router(launches.router)


@app.get("/")
def read_root():
    return {"status": "ok"}
