# Born2beRoot Project Guide - Expanded

## Step 1: Choose and Install the Operating System

1. Choose between Debian (recommended for beginners) or Rocky Linux.
   - Debian: Download the latest stable version from https://www.debian.org/
   - Rocky Linux: Download the latest stable version from https://rockylinux.org/

2. Set up VirtualBox (or UTM for Mac M1 users):
   - Download and install VirtualBox from https://www.virtualbox.org/
   - For Mac M1 users, download UTM from https://mac.getutm.app/

3. Create a new virtual machine:
   - Open VirtualBox and click "New"
   - Name your VM (e.g., "Born2beRoot")
   - Choose "Linux" as the type and select "Debian" or "Red Hat" (for Rocky) as the version
   - Allocate at least 1024MB of RAM
   - Create a new virtual hard disk (VDI, dynamically allocated, at least 8GB)

4. Install the OS:
   - Start the VM and select your ISO file as the boot medium
   - Choose "Install" (not "Graphical Install")
   - Follow the installation prompts, ensuring you don't install a desktop environment or any unnecessary software

## Step 2: Configure Partitions

1. During installation, when you reach the partitioning step, choose "Manual":
   - Create a new partition table if prompted
   - Create the following partitions:
     a. /boot (non-encrypted, 500MB)
     b. Encrypted volume for LVM (rest of the space)

2. Set up LVM within the encrypted volume:
   - Create a volume group (e.g., "LVMGroup")
   - Create logical volumes:
     - root (/, at least 10GB)
     - swap (2GB or equal to your RAM size)
     - home (/home, remaining space)

3. Format the partitions:
   - /boot as ext4
   - root (/) as ext4
   - home (/home) as ext4
   - swap as swap

4. Set mount points for each partition

## Step 3: Basic System Configuration

1. Set the hostname:
   - During installation, set the hostname to your login ending with 42 (e.g., jsmith42)
   - After installation, you can change it with: `sudo hostnamectl set-hostname yourhostname42`

2. Install and configure sudo:
   - Log in as root
   - Run `apt install sudo` (Debian) or `dnf install sudo` (Rocky)
   - Add your user to the sudo group: `usermod -aG sudo yourusername`

3. Install and configure SSH:
   - Install SSH: `sudo apt install openssh-server` (Debian) or `sudo dnf install openssh-server` (Rocky)
   - Edit the SSH config file: `sudo nano /etc/ssh/sshd_config`
   - Change the default port to 4242: `Port 4242`
   - Disable root login: `PermitRootLogin no`
   - Restart SSH service: `sudo systemctl restart sshd`

4. Configure the firewall:
   - For Debian (UFW):
     ```
     sudo apt install ufw
     sudo ufw enable
     sudo ufw allow 4242
     ```
   - For Rocky (firewalld):
     ```
     sudo systemctl start firewalld
     sudo systemctl enable firewalld
     sudo firewall-cmd --add-port=4242/tcp --permanent
     sudo firewall-cmd --reload
     ```

## Step 4: Implement Strong Password Policy

1. Edit password aging policies:
   - Open the file: `sudo nano /etc/login.defs`
   - Modify the following lines:
     ```
     PASS_MAX_DAYS 30
     PASS_MIN_DAYS 2
     PASS_WARN_AGE 7
     ```

2. Install and configure password quality checking library:
   - Install the library: `sudo apt install libpam-pwquality` (Debian) or `sudo dnf install libpwquality` (Rocky)
   - Edit the PAM config: `sudo nano /etc/pam.d/common-password`
   - Add or modify the password quality line:
     ```
     password requisite pam_pwquality.so retry=3 minlen=10 ucredit=-1 lcredit=-1 dcredit=-1 maxrepeat=3 usercheck=1 difok=7 enforce_for_root
     ```

3. Change all existing user passwords:
   - For each user (including root): `sudo passwd username`

## Step 5: Configure Sudo

1. Create a sudo configuration file:
   `sudo visudo -f /etc/sudoers.d/sudoconfig`

2. Add the following lines to the file:
   ```
   Defaults        passwd_tries=3
   Defaults        badpass_message="Custom error message for wrong sudo password"
   Defaults        logfile="/var/log/sudo/sudo.log"
   Defaults        log_input,log_output
   Defaults        requiretty
   Defaults        secure_path="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/snap/bin"
   ```

3. Create the sudo log directory:
   `sudo mkdir -p /var/log/sudo`

## Step 6: Create monitoring.sh Script

1. Create the script file:
   `sudo nano /usr/local/bin/monitoring.sh`

2. Add the following content to the script:
   ```bash
   #!/bin/bash
   arc=$(uname -a)
   pcpu=$(grep "physical id" /proc/cpuinfo | sort | uniq | wc -l)
   vcpu=$(grep "^processor" /proc/cpuinfo | wc -l)
   fram=$(free -m | awk '$1 == "Mem:" {print $2}')
   uram=$(free -m | awk '$1 == "Mem:" {print $3}')
   pram=$(free | awk '$1 == "Mem:" {printf("%.2f"), $3/$2*100}')
   fdisk=$(df -BG | grep '^/dev/' | grep -v '/boot$' | awk '{ft += $2} END {print ft}')
   udisk=$(df -BM | grep '^/dev/' | grep -v '/boot$' | awk '{ut += $3} END {print ut}')
   pdisk=$(df -BM | grep '^/dev/' | grep -v '/boot$' | awk '{ut += $3} {ft+= $2} END {printf("%d"), ut/ft*100}')
   cpul=$(top -bn1 | grep '^%Cpu' | cut -c 9- | xargs | awk '{printf("%.1f%%"), $1 + $3}')
   lb=$(who -b | awk '$1 == "system" {print $3 " " $4}')
   lvmu=$(if [ $(lsblk | grep "lvm" | wc -l) -eq 0 ]; then echo no; else echo yes; fi)
   ctcp=$(ss -neopt state established | wc -l)
   ulog=$(users | wc -w)
   ip=$(hostname -I)
   mac=$(ip link show | grep "ether" | awk '{print $2}')
   cmds=$(journalctl _COMM=sudo | grep COMMAND | wc -l)
   wall "    #Architecture: $arc
    #CPU physical: $pcpu
    #vCPU: $vcpu
    #Memory Usage: $uram/${fram}MB ($pram%)
    #Disk Usage: $udisk/${fdisk}Gb ($pdisk%)
    #CPU load: $cpul
    #Last boot: $lb
    #LVM use: $lvmu
    #Connections TCP: $ctcp ESTABLISHED
    #User log: $ulog
    #Network: IP $ip ($mac)
    #Sudo: $cmds cmd"
   ```

3. Make the script executable:
   `sudo chmod +x /usr/local/bin/monitoring.sh`

4. Configure cron to run the script every 10 minutes:
   - Open the crontab file: `sudo crontab -e`
   - Add the following line:
     ```
     */10 * * * * /usr/local/bin/monitoring.sh
     ```

## Step 7: Final Checks and Preparation

1. Ensure all services are running correctly:
   - Check SSH: `sudo systemctl status sshd`
   - Check firewall: `sudo ufw status` (Debian) or `sudo firewall-cmd --list-all` (Rocky)

2. Verify password policies:
   - Try creating a user with a weak password
   - Check password expiration: `sudo chage -l username`

3. Test sudo configuration:
   - Run a command with sudo and check the log file

4. Test the monitoring script:
   - Run it manually: `sudo /usr/local/bin/monitoring.sh`
   - Wait for the cron job to execute it

5. Practice explaining the script and how to interrupt it (you can use `ctrl+c` to stop the script if running manually)

## Step 8: Create signature.txt

1. Locate your virtual machine's disk file:
   - Usually in `~/VirtualBox VMs/[VM_NAME]/` for VirtualBox
   - For UTM, check `~/Library/Containers/com.utmapp.UTM/Data/Documents/`

2. Generate the SHA1 hash:
   - For VirtualBox (replace [VM_NAME] with your VM's name):
     - On Linux/macOS: `shasum ~/VirtualBox\ VMs/[VM_NAME]/[VM_NAME].vdi`
     - On Windows: `certUtil -hashfile %HOMEDRIVE%%HOMEPATH%\VirtualBox VMs\[VM_NAME]\[VM_NAME].vdi sha1`
   - For UTM on Mac M1:
     `shasum ~/Library/Containers/com.utmapp.UTM/Data/Documents/[VM_NAME].utm/Images/disk-0.qcow2`

3. Create signature.txt:
   - Create a new file named `signature.txt` in your Git repository
   - Paste only the hash (not the filename) into this file
