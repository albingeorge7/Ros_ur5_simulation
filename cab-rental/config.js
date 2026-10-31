// Edit these (or set the matching environment variables) to describe your service.
const env = process.env;

export default {
  port: Number(env.PORT || 3000),
  baseUrl: env.BASE_URL || '',            // public URL, used in bill links e.g. https://mycab.example.com
  timezone: env.TIMEZONE || 'Asia/Kolkata',
  currency: env.CURRENCY || '₹',
  business: {
    name: env.BUSINESS_NAME || 'City Ride Cabs',
    tagline: env.TAGLINE || 'Clean, comfortable and on-time rides',
    driverName: env.DRIVER_NAME || 'Your Driver Name',
    phone: env.DRIVER_PHONE || '+91 90000 00000',
    whatsapp: env.DRIVER_WHATSAPP || '919000000000', // digits only, country code first
    email: env.DRIVER_EMAIL || '',
    address: env.DRIVER_ADDRESS || '12, Main Road, Your City - 000000',
    vehicle: env.VEHICLE || 'Sedan (AC) • 4 passengers + luggage',
    licence: env.CAB_NUMBER || 'XX 00 XX 0000',
    hours: env.HOURS || 'Available 24 x 7 (advance booking recommended)',
    areas: (env.SERVICE_AREAS || 'City rides,Airport transfers,Outstation trips,Hourly rental').split(','),
    ratePerKm: Number(env.RATE_PER_KM || 14),
    baseFare: Number(env.BASE_FARE || 50),
    taxPercent: Number(env.TAX_PERCENT || 0),
  },
  // Driver login. CHANGE THE PASSWORD before going live (DRIVER_PASSWORD=...).
  driverUser: env.DRIVER_USER || 'driver',
  driverPassword: env.DRIVER_PASSWORD || 'driver123',
  // Optional phone alerts via Telegram bot (create one with @BotFather).
  telegram: { token: env.TELEGRAM_BOT_TOKEN || '', chatId: env.TELEGRAM_CHAT_ID || '' },
  // Optional email for bills: install nodemailer (npm i) and set SMTP_*.
  smtp: { host: env.SMTP_HOST || '', port: Number(env.SMTP_PORT || 587), user: env.SMTP_USER || '', pass: env.SMTP_PASS || '', from: env.SMTP_FROM || '' },
};
