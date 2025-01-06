from functools import lru_cache
from contextlib import asynccontextmanager
from typing import Annotated

from fastapi import Depends, FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse

from app.routers import sensors
from app.config.settings import APISettings

# To fix issue with CORS when requesting on the browser
origins = [
    "http://localhost",
    "http://localhost:8080",
    "http://localhost:5173",
]


@asynccontextmanager
async def lifespan(app: FastAPI):
    print("Before Start FASTAPI")
    yield
    print("After End FASTAPI")


@lru_cache
def get_settings():
    return APISettings()  # type: ignore


app = FastAPI(
    debug=True,
    lifespan=lifespan,
)


app.include_router(sensors.router)

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/")
async def info(settings: Annotated[APISettings, Depends(get_settings)]):
    return HTMLResponse(
        f"<h1>API is up and running. Version of the API: {settings.version}</h1>"
    )
