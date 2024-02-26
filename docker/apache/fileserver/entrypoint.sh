#!/bin/bash
set -e

# Remove any stale PID or socket files before starting Apache
rm -f /var/run/apache2/apache2.pid
rm -f /var/run/apache2/cgisock.*

apache2ctl -D FOREGROUND &

# Make sure we can edit files on mapped volumes
chown -R www-data:www-data /var/www/filegator/repository
chmod -R 775  /var/www/filegator/repository

exec "$@"
