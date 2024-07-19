import { Component, Input, OnInit } from '@angular/core';
import { SocketClientService } from '../../services/socket-client/socket-client.service';
import { MissionData } from 'src/app/interfaces/MissionData';
import { MatDialog } from '@angular/material/dialog';
import { XtermComponent } from '../xterm/xterm.component';

@Component({
  selector: 'app-mission-history',
  templateUrl: './mission-history.component.html',
  styleUrl: './mission-history.component.css'
})
export class MissionHistoryComponent implements OnInit {
  @Input() showOptions = true;

  displayedColumns: string[] = [
    'missionId',
    'date',
    'time',
    'type',
    'robots',
    'options'
  ];

  missions: MissionData[] = [];

  constructor(private socketClient: SocketClientService, private dialog: MatDialog) { }

  ngOnInit(): void {
    if (!this.showOptions) {
      this.displayedColumns.pop();
    }
    this.getMissionHistory();
    this.initializeSocketListeners();
  }

  get currentDate(): string {
    const options: Intl.DateTimeFormatOptions = {
      day: 'numeric',
      month: 'long',
      year: 'numeric'
    };
    return new Intl.DateTimeFormat('fr-FR', options).format(new Date());
  }

  getRobotColor(robot: string): string {
    if (robot === 'Aucun robot') {
      return 'red';
    }
    const physicalRobotColors = ['blue', 'green'];
    const simulationRobotColors = ['purple', 'orange'];
    if (robot.startsWith('limo_')) {
      const robotNumber = parseInt(robot.replace('limo_', ''), 10);
      return simulationRobotColors[robotNumber % simulationRobotColors.length];
    }
    switch (robot) {
      case 'lm1166':
        return physicalRobotColors[0];
      case 'lm1342':
        return physicalRobotColors[1];
      default:
        return 'black';
    }
  }

  openMissionInfo(missionId: number): void {
    const dialogRef = this.dialog.open(XtermComponent, {
      width: '60vw',
      height: '60vh',
      data: { missionId: missionId }
    });
  }

  deleteMission(missionId: number): void {
    this.socketClient.socket.emit('delete_mission', missionId);
  }

  private getMissionHistory(): void {
    this.socketClient.socket.on('mission_history', (missions: MissionData[]) => {
      this.missions = missions.sort((a, b) => b.missionId - a.missionId);
    });
    this.socketClient.socket.emit('request_history');
  }

  private initializeSocketListeners(): void {
    this.socketClient.socket.on('last_mission', (lastMission: MissionData) => {
      this.missions = [lastMission, ...this.missions];
    });
  }
}
