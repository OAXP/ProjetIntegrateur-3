import { Component, ElementRef, ViewChild, AfterViewInit, OnDestroy } from '@angular/core';
import { RobotInfo } from 'src/app/interfaces/RobotInfo';
import { CommunicationService } from 'src/app/services/communication/communication.service';
import { InfoService } from 'src/app/services/info/info.service';

export const MIN_WIDTH = 475;

@Component({
  selector: 'app-locate',
  templateUrl: './locate.component.html',
  styleUrls: ['./locate.component.css']
})
export class LocateComponent implements AfterViewInit, OnDestroy {
  @ViewChild('container') container!: ElementRef;
  isNarrow = false;
  private resizeObserver!: ResizeObserver;

  constructor(private communicationService: CommunicationService, private infoService: InfoService){}

  ngAfterViewInit(): void {
    this.resizeObserver = new ResizeObserver(entries => {
      for (let entry of entries) {
        this.isNarrow = entry.contentRect.width < MIN_WIDTH;
      }
    });

    this.resizeObserver.observe(this.container.nativeElement);
  }

  ngOnDestroy(): void {
    if (this.resizeObserver) {
      this.resizeObserver.disconnect();
    }
  }

  identify(robot_id = 'all') {
    return this.communicationService.identify(robot_id).subscribe();
  }

  getRobotInfos() {
    return Array.from(this.infoService.robotInfos);
  }

  getRobotInfoById(robotId: string): RobotInfo | undefined {
    return this.infoService.robotInfos.get(robotId);
  }
}