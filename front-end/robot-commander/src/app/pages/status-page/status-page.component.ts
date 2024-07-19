import { Component } from '@angular/core';
import { GridsterConfig, GridsterItem } from 'angular-gridster2';
import { MatSnackBar } from '@angular/material/snack-bar';
import { Subscription } from 'rxjs';
import { SocketClientService } from 'src/app/services/socket-client/socket-client.service';
import { DEFAULT_DASHBOARD_LAYOUT } from 'src/app/utils/constants';

@Component({
  selector: 'app-status-page',
  templateUrl: './status-page.component.html',
  styleUrls: ['./status-page.component.css']
})
export class StatusPageComponent {
  isLocked = true;
  lockState = this.isLocked ? 'lock' : 'unlock';
  title = 'robot-commander';
  options!: GridsterConfig;
  dashboard!: Array<GridsterItem>;
  lastSnackbarMessage = '';
  private connectSubscription!: Subscription;

  constructor(private socketClientService: SocketClientService, private snackBar: MatSnackBar) { }

  ngOnInit() {
    this.connectSubscription = this.socketClientService.OnConnectionChanged.subscribe((state) => {
      if (state) {
        this.snackBar.open('Connexion établie!', 'Fermer', {
          panelClass: ['success-snackbar'],
          duration: 2000
        });
        this.lastSnackbarMessage = 'Connexion établie!';
      } else if (this.lastSnackbarMessage !== 'Déconnecté') {
        this.snackBar.open('Déconnecté', 'Fermer', {
          panelClass: ['error-snackbar'],
          duration: 2000
        });
        this.lastSnackbarMessage = 'Déconnecté';
      }
    });

    this.options = {
      minCols: 8,
      minRows: 7,
      draggable: {
        enabled: !this.isLocked
      },
      resizable: {
        enabled: !this.isLocked
      },
      itemChangeCallback: () => {
        this.saveDashboardState();
      },
      itemResizeCallback: () => {
        this.saveDashboardState();
      }
    };

    const savedDashboard = localStorage.getItem('dashboard');
    if (savedDashboard) {
      this.dashboard = JSON.parse(savedDashboard);
    } else {
      this.dashboard = DEFAULT_DASHBOARD_LAYOUT;
    }
  }

  updateLockState(lockState: boolean) {
    this.isLocked = lockState;
    this.options.draggable!.enabled = !this.isLocked;
    this.options.resizable!.enabled = !this.isLocked;
    this.options = { ...this.options };
  }

  removeItem(item: GridsterItem) {
    this.dashboard.splice(this.dashboard.indexOf(item), 1);
  }

  saveDashboardState() {
    localStorage.setItem('dashboard', JSON.stringify(this.dashboard));
  }

  ngOnDestroy() {
    if (this.connectSubscription) {
      this.connectSubscription.unsubscribe();
    }
  }
}
