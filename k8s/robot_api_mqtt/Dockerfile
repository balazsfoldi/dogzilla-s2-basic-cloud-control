FROM python:3.12-slim

# Rendszer csomagok
RUN pip install --no-cache-dir --upgrade pip

# Munkakönyvtár a konténerben
WORKDIR /app

# Függőségek
COPY req.txt .
RUN pip install --no-cache-dir -r req.txt

# Alkalmazás kód
COPY main.py .

# FastAPI + Uvicorn induljon
EXPOSE 8000

CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
