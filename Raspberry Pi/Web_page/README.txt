Getting the Raspberry Pi 2 Ready
by Lourdes Morales

To get the Raspberry Pi 2 ready for the webpage and 
to enable control of the GPIOs from there we had to
do the following:

1. Install APACHE2 (make the Raspberry Pi 2 a web server)

	sudo apt-get install apache2 -y

2. Install PHP5 (to allow us to run php files)

	sudo apt-get install php5 libapache2-mod-php5 -y

3. Change permissions (so that we could run the commands 
to run python code without having to ssh-in)

	sudo visudo

	Add this line to the end of the file:

		www-data ALL=(ALL) NOPASSWD: ALL

	Press CTRL + X to save and exit.

	Restart Apache

		sudo /etc/init.d/apache2 restart

4. Finally, upload the files for the web page onto the 
directory /var/www/html (and removed the default 
index.html file that was there)