import { Component } from '@angular/core';
import { CommunicationService } from 'src/app/services/communication/communication.service';

@Component({
  selector: 'app-mission-control',
  templateUrl: './mission-control.component.html',
  styleUrl: './mission-control.component.css'
})

export class MissionControlComponent {

  constructor(private communicationService: CommunicationService) { }

  startMission() {
    return this.communicationService.startMission().subscribe();
  }

  stopMission() {
    return this.communicationService.stopMission().subscribe();
  }
}
