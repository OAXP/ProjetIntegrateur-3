import { Component, OnInit } from '@angular/core';
import { Router, NavigationEnd } from '@angular/router';
import { Subscription } from 'rxjs';
import { SocketClientService } from 'src/app/services/socket-client/socket-client.service';
import { MatSnackBar } from '@angular/material/snack-bar';
import { InfoService } from 'src/app/services/info/info.service';
import { MatMenuTrigger } from '@angular/material/menu';
import { ViewChild } from '@angular/core';
import { MIN_VOLTAGE, MAX_VOLTAGE } from 'src/app/utils/constants';

@Component({
  selector: 'app-root',
  templateUrl: './app.component.html',
  styleUrls: ['./app.component.css']
})
export class AppComponent implements OnInit {
  @ViewChild(MatMenuTrigger) menuTrigger!: MatMenuTrigger;
  private connectSubscription!: Subscription;
  connected = false;
  robotVoltageMap: { [robot_id: string]: number } = {};
  showOptions = true;

  get connectionStatusIcon(): string {
    return this.connected ? 'link' : 'link_off';
  }

  get voltageStatusIcon(): string {
    return this.allRobotsCharged() ? 'battery_charging_full' : 'battery_alert';
  }

  get robotVoltagesArray() {
    return Object.entries(this.robotVoltageMap).map(([robot_id, voltage]) => ({
      robot_id,
      voltage
    }));
  }

  get batteryIcon(): string {
    const averagePercentage = this.averageBatteryPercentage();
    if (averagePercentage >= 90) return 'battery_full';
    if (averagePercentage >= 60) return 'battery_6_bar';
    if (averagePercentage >= 30) return 'battery_3_bar';
    if (averagePercentage >= 10) return 'battery_1_bar';
    return 'battery_alert';
  }

  get batteryColorClass(): string {
    const averagePercentage = this.averageBatteryPercentage();
    if (averagePercentage >= 60) return 'battery-high';
    if (averagePercentage >= 30) return 'battery-medium';
    return 'battery-low';
  }

  constructor(private socketClientService: SocketClientService, private router: Router, private snackBar: MatSnackBar, private infoService: InfoService) { }

  ngOnInit() {
    this.checkExistingConnection();
    this.connectSubscription = this.socketClientService.OnConnectionChanged.subscribe((state) => {
      this.connected = state;
    });

    this.infoService.robotInfos.forEach((info, robot_id) => {
      this.robotVoltageMap[robot_id] = info.battery_voltage;
    });

    this.router.events.subscribe(event => {
      if (event instanceof NavigationEnd) {
        this.showOptions = !event.urlAfterRedirects.includes('login');
      }
    });

    setInterval(() => {
      this.infoService.robotInfos.forEach((info, robot_id) => {
        this.robotVoltageMap[robot_id] = info.battery_voltage;
      });
    }, 1000);
  }

  averageBatteryPercentage(): number {
    const percentages = this.robotVoltagesArray.map(({ voltage }) =>
      this.convertVoltageToPercentage(voltage)
    );
    const sum = percentages.reduce((a, b) => a + b, 0);
    return percentages.length > 0 ? sum / percentages.length : 0;
  }

  convertVoltageToPercentage(voltage: number): number {
    return ((voltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE)) * 100;
  }

  showConnectionStatus() {
    if (this.connected) {
      this.snackBar.open('Connecté au serveur', 'Fermer', {
        panelClass: ['success-snackbar'],
        duration: 2000
      });
    } else {
      this.snackBar.open('Déconnecté du serveur', 'Fermer', {
        panelClass: ['error-snackbar'],
        duration: 2000
      });
    }
  }

  allRobotsCharged(): boolean {
    return Object.values(this.robotVoltageMap).every(voltage => voltage >= 11.5) && Object.values(this.robotVoltageMap).length > 0;
  }

  isActive(url: string): boolean {
    return this.router.url.includes(url);
  }

  logout() {
    this.socketClientService.disconnect();
    this.router.navigate(['/login']);
  }

  checkExistingConnection() {
    const username = localStorage.getItem('username');
    if (username) {
      this.socketClientService.connectWithUsername(username);
    }
  }
}
