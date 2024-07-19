import { Component, OnInit, OnDestroy, ChangeDetectorRef } from '@angular/core';
import { RobotInfo } from 'src/app/interfaces/RobotInfo';
import { InfoService } from 'src/app/services/info/info.service';

@Component({
  selector: 'app-system-runtime',
  templateUrl: './system-runtime.component.html',
  styleUrls: ['./system-runtime.component.css']
})
export class SystemRuntimeComponent implements OnInit, OnDestroy {
  private robotTimers: { [robotId: string]: any } = {};
  elapsedTimes: { [robotId: string]: string } = {};

  constructor(private infoService: InfoService, private cdRef: ChangeDetectorRef) { }

  ngOnInit(): void {
    this.infoService.robotInfosObservable.subscribe((updatedRobots) => {
      this.checkRobotStatus(updatedRobots);
    });
  }

  ngOnDestroy(): void {
    Object.values(this.robotTimers).forEach(timer => clearInterval(timer));
  }

  getRobotIds(): string[] {
    return Object.keys(this.elapsedTimes);
  }

  checkRobotStatus(robots: Map<string, RobotInfo>) {
    robots.forEach((robotInfo, robotId) => {
      if (!this.elapsedTimes[robotId]) {
        this.elapsedTimes[robotId] = "00:00:00";
        this.triggerChangeDetection();
      }

      if (robotInfo.start_time === 0.0) {
        if (this.robotTimers[robotId]) {
          clearInterval(this.robotTimers[robotId]);
          delete this.robotTimers[robotId];
        }
      } else {
        if (!this.robotTimers[robotId]) {
          this.robotTimers[robotId] = setInterval(() => {
            const currentTime = Date.now() / 1000;
            const elapsedTime = currentTime - robotInfo.start_time;
            const totalMinutes = Math.floor(elapsedTime / 60);
            const minutes = (totalMinutes % 60).toString().padStart(2, '0');
            const seconds = Math.floor(elapsedTime % 60).toString().padStart(2, '0');
            const centiseconds = Math.floor((elapsedTime % 1) * 100).toString().padStart(2, '0');
            this.elapsedTimes[robotId] = `${minutes}:${seconds}:${centiseconds}`;
            this.triggerChangeDetection();
          }, 10);
        }
      }
    });
  }

  triggerChangeDetection() {
    this.cdRef.detectChanges();
  }
}
