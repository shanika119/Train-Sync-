import os
import json
import copy
from flask import Flask, request, jsonify, send_file
from flask_mail import Mail, Message
import qrcode
from io import BytesIO
from dotenv import load_dotenv
from flask_cors import CORS
from reportlab.pdfgen import canvas
from reportlab.lib.pagesizes import letter
from reportlab.lib.utils import ImageReader

load_dotenv()

app = Flask(__name__)
CORS(app)

# Flask-Mail config
app.config['MAIL_SERVER'] = os.getenv('MAIL_SERVER', 'smtp.gmail.com')
app.config['MAIL_PORT'] = int(os.getenv('MAIL_PORT', 587))
app.config['MAIL_USE_TLS'] = os.getenv('MAIL_USE_TLS', 'True').lower() in ('true', '1', 't')
app.config['MAIL_USERNAME'] = os.getenv('MAIL_USERNAME')
app.config['MAIL_PASSWORD'] = os.getenv('MAIL_PASSWORD')
app.config['MAIL_DEFAULT_SENDER'] = os.getenv('MAIL_DEFAULT_SENDER')

mail = Mail(app)

# Admin login defaults
ADMIN_USERNAME = os.getenv('ADMIN_USERNAME', 'admin')
ADMIN_PASSWORD = os.getenv('ADMIN_PASSWORD', 'adminpass')

INTERMEDIATE_STATION = 'Colombo Fort'
TRAIN_DATA_FILE = 'train_data.json'
train_data = {}

CENTRAL_SOUTH_CITIES = ['Badulla', 'Kandy', 'Nuwara Eliya', 'Colombo Fort']
NORTH_CULTURAL_CITIES = ['Jaffna', 'Anuradhapura', 'Polonnaruwa', 'Colombo Fort']
ALL_CITIES = sorted(list(set(CENTRAL_SOUTH_CITIES + NORTH_CULTURAL_CITIES)))


def generate_all_unique_directional_routes():
    routes = set()
    n_cities = len(ALL_CITIES)
    for i in range(n_cities):
        for j in range(n_cities):
            if i != j:
                from_city_normalized = ALL_CITIES[i].replace(' ', '-')
                to_city_normalized = ALL_CITIES[j].replace(' ', '-')
                routes.add(f"{from_city_normalized}-{to_city_normalized}")
    return sorted(list(routes))


def load_train_data():
    global train_data
    if os.path.exists(TRAIN_DATA_FILE):
        with open(TRAIN_DATA_FILE, 'r') as f:
            train_data = json.load(f)
    else:
        train_data = {
            "Badulla-Colombo-Fort": [
                {"id": "RT001", "name": "Udarata Menike", "time": "06:00", "price_1st_class": 2500, "price_2nd_class": 1500},
                {"id": "RT002", "name": "Podi Menike", "time": "08:30", "price_1st_class": 2300, "price_2nd_class": 1400},
            ],
            "Colombo-Fort-Jaffna": [
                {"id": "RT003", "name": "Yal Devi", "time": "05:45", "price_1st_class": 3000, "price_2nd_class": 2000},
                {"id": "RT004", "name": "Uththara Devi", "time": "09:50", "price_1st_class": 2800, "price_2nd_class": 1900},
            ]
        }

    all_possible_routes = generate_all_unique_directional_routes()
    for route in all_possible_routes:
        if route not in train_data:
            train_data[route] = []


def save_train_data(data):
    with open(TRAIN_DATA_FILE, 'w') as f:
        json.dump(data, f, indent=4)


@app.route('/api/trains', methods=['GET'])
def get_trains():
    return jsonify(train_data)


@app.route('/api/tracking/trains', methods=['GET'])
def get_tracking_trains():
    """Get all trains formatted for tracking selection"""
    all_trains = []
    for route, trains in train_data.items():
        for train in trains:
            route_parts = route.split('-')
            from_station = route_parts[0].replace('-', ' ')
            to_station = '-'.join(route_parts[1:]).replace('-', ' ')

            all_trains.append({
                'id': train['id'],
                'name': train['name'],
                'time': train['time'],
                'route': route,
                'from': from_station,
                'to': to_station,
                'displayRoute': f"{from_station} → {to_station}",
                'isActive': True
            })
    return jsonify(all_trains)


@app.route('/api/admin/login', methods=['POST'])
def admin_login():
    data = request.get_json()
    username = data.get('username')
    password = data.get('password')

    if username == ADMIN_USERNAME and password == ADMIN_PASSWORD:
        return jsonify({'message': 'Login successful!'}), 200
    else:
        return jsonify({'error': 'Invalid credentials'}), 401


@app.route('/api/trains', methods=['POST'])
def add_train():
    data = request.get_json()
    route_name = data.get('route')
    train = data.get('train')

    if not route_name or not train:
        return jsonify({'error': 'Route name and train data are required'}), 400

    if route_name not in train_data:
        train_data[route_name] = []

    if any(t['id'] == train['id'] for t in train_data[route_name]):
        return jsonify({'error': 'Train with this ID already exists in this route'}), 409

    train_data[route_name].append(train)
    save_train_data(train_data)
    return jsonify({'message': 'Train added successfully!'}), 201


@app.route('/api/trains/<route_name>/<train_id>', methods=['PUT'])
def update_train(route_name, train_id):
    data = request.get_json()

    if route_name not in train_data:
        return jsonify({'error': 'Route not found'}), 404

    for i, train in enumerate(train_data[route_name]):
        if train['id'] == train_id:
            train_data[route_name][i].update(data)
            save_train_data(train_data)
            return jsonify({'message': 'Train updated successfully!'}), 200

    return jsonify({'error': 'Train not found in this route'}), 404


@app.route('/api/trains/<route_name>/<train_id>', methods=['DELETE'])
def delete_train(route_name, train_id):
    global train_data
    if route_name not in train_data:
        return jsonify({'error': 'Route not found'}), 404

    initial_len = len(train_data[route_name])
    train_data[route_name] = [t for t in train_data[route_name] if t['id'] != train_id]

    if len(train_data[route_name]) < initial_len:
        save_train_data(train_data)
        return jsonify({'message': 'Train deleted successfully!'}), 200
    else:
        return jsonify({'error': 'Train not found in this route'}), 404


@app.route('/api/send-email', methods=['POST'])
def send_email():
    try:
        data = request.get_json()
        booking_details = data.get('booking_details', {})
        recipient_email = data.get('recipient_email')

        # Generate QR for only train name and date
        train_name = booking_details.get('selectedTrainName', 'Unknown Train')
        date = booking_details.get('selectedDate', 'Unknown Date')
        qr_data_string = f"Train: {train_name}\nDate: {date}"

        if not recipient_email:
            return jsonify({'error': 'Recipient email is required'}), 400

        # Generate large QR code
        qr = qrcode.QRCode(
            version=1,
            error_correction=qrcode.constants.ERROR_CORRECT_H,
            box_size=20,
            border=4,
        )
        qr.add_data(qr_data_string)
        qr.make(fit=True)
        qr_img = qr.make_image(fill_color="black", back_color="white")

        qr_buffered = BytesIO()
        qr_img.save(qr_buffered, format="PNG", optimize=True)
        qr_buffered.seek(0)
        qr_data_for_attachment = qr_buffered.read()
        qr_buffered.seek(0)

        # Generate PDF
        pdf_buffer = BytesIO()
        c = canvas.Canvas(pdf_buffer, pagesize=letter)
        c.setFont("Helvetica-Bold", 24)
        c.drawString(50, 750, "EcoDuino - Your E-Ticket")

        qr_image = ImageReader(qr_buffered)
        c.drawImage(qr_image, 50, 500, width=250, height=250)  # Larger QR

        c.setFont("Helvetica", 12)
        y_position = 470
        line_height = 20
        details_map = {
            "Email": booking_details.get('email'),
            "Date": booking_details.get('selectedDate'),
            "From": booking_details.get('fromStation'),
            "To": booking_details.get('toStation'),
            "Train": f"{booking_details.get('selectedTrainName')} ({booking_details.get('selectedTrainTime')})",
            "Class": booking_details.get('selectedClass'),
            "Seats": ", ".join(map(str, booking_details.get('selectedSeats', []))),
            "Total Cost": f"Rs. {booking_details.get('totalCost', 0):.2f}"
        }

        for label, value in details_map.items():
            c.drawString(50, y_position, f"{label}: {value}")
            y_position -= line_height

        c.showPage()
        c.save()
        pdf_buffer.seek(0)

        msg = Message(
            'Your EcoDuino Booking Confirmation',
            sender=app.config['MAIL_DEFAULT_SENDER'],
            recipients=[recipient_email]
        )

        msg.body = f"""Dear valued customer,

Thank you for booking your train ticket with EcoDuino!

Train: {train_name}
Date: {date}

Please find your QR code and a PDF e-ticket attached.

Best regards,
The EcoDuino Team
"""

        msg.attach("EcoDuino_QR_Code.png", "image/png", qr_data_for_attachment)
        msg.attach("EcoDuino_Train_Ticket.pdf", "application/pdf", pdf_buffer.read())

        mail.send(msg)
        return jsonify({'message': 'Booking confirmation email sent successfully!'}), 200

    except Exception as e:
        return jsonify({'error': f'Error sending email: {str(e)}'}), 500


@app.route('/api/download-qr-code', methods=['POST'])
def download_qr_code():
    data = request.get_json()
    booking_details = data.get('booking_details', {})

    # Generate QR for only train name and date
    train_name = booking_details.get('selectedTrainName', 'Unknown Train')
    date = booking_details.get('selectedDate', 'Unknown Date')
    qr_data_string = f"Train: {train_name}\nDate: {date}"

    try:
        qr = qrcode.QRCode(
            version=1,
            error_correction=qrcode.constants.ERROR_CORRECT_H,
            box_size=20,
            border=4,
        )
        qr.add_data(qr_data_string)
        qr.make(fit=True)
        qr_img = qr.make_image(fill_color="black", back_color="white")

        buffered = BytesIO()
        qr_img.save(buffered, format="PNG", optimize=True)
        buffered.seek(0)

        return send_file(
            buffered,
            mimetype='image/png',
            as_attachment=True,
            download_name='EcoDuino_train_ticket_qr_code.png'
        )

    except Exception as e:
        return jsonify({'error': f'Failed to generate QR code: {str(e)}'}), 500


if __name__ == '__main__':
    load_train_data()
    app.run(debug=True)
