import logging

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from routers import subsystems, launches, term

logger = logging.getLogger(__name__)

app = FastAPI()
app.logger = logger

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(subsystems.router)
app.include_router(launches.router)
app.include_router(term.router)


@app.get("/")
def read_root():
    return {"status": "ok"}
