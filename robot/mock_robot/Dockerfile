FROM python:3.11-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install -r requirements.txt

COPY mock_robot.py .

CMD ["python", "mock_robot.py"]