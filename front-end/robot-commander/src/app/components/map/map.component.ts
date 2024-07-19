import { Component, OnDestroy, Input, AfterViewInit, ElementRef, ViewChild } from '@angular/core';
import { Router } from '@angular/router';
import { InfoService } from 'src/app/services/info/info.service';
import { Subscription } from 'rxjs';

@Component({
  selector: 'app-map',
  templateUrl: './map.component.html',
  styleUrl: './map.component.css'
})
export class MapComponent implements AfterViewInit, OnDestroy {
  @Input() isWidget = false;
  private resizeObserver!: ResizeObserver;
  @ViewChild('mapContainer') mapContainer!: ElementRef;
  @ViewChild('matCard', { read: ElementRef, static: false }) matCard?: ElementRef;
  @ViewChild('canvasElement') canvasElement!: ElementRef<HTMLCanvasElement>;
  isLoading = true;
  private mapInfoSubscription!: Subscription;

  constructor(private infoService: InfoService, private router: Router) { }

  ngAfterViewInit(): void {
    this.setupResizeObserver();
    this.mapInfoSubscription = this.infoService.mapInfoObservable.subscribe({
      next: (imageDataUrl) => {
        this.drawImageOnCanvas(imageDataUrl);
      }
    });
  }

  ngOnDestroy(): void {
    if (this.resizeObserver) {
      this.resizeObserver.disconnect();
    }
    if (this.mapInfoSubscription) {
      this.mapInfoSubscription.unsubscribe();
    }
  }

  openMap() {
    this.router.navigate(['/map']);
  }

  drawImageOnCanvas(imageDataUrl: string): void {
    const context = this.canvasElement.nativeElement.getContext('2d');
    const container = this.mapContainer.nativeElement;
  
    const containerWidth = container.offsetWidth;
    const containerHeight = container.offsetHeight;
  
    const minSize = Math.min(containerWidth, containerHeight);
  
    if (context) {
      const image = new Image();
      image.onload = () => {
        this.canvasElement.nativeElement.width = minSize;
        this.canvasElement.nativeElement.height = minSize;
  
        const scale = Math.max(minSize / image.width, minSize / image.height);
        const imageWidthScaled = image.width * scale;
        const imageHeightScaled = image.height * scale;
        const dx = (minSize - imageWidthScaled) / 2;
        const dy = (minSize - imageHeightScaled) / 2;
  
        context.clearRect(0, 0, context.canvas.width, context.canvas.height);
        context.drawImage(image, dx, dy, imageWidthScaled, imageHeightScaled);
  
        this.isLoading = false;
      };
      image.src = imageDataUrl;
    }
  }

  private setupResizeObserver(): void {
    this.resizeObserver = new ResizeObserver(entries => {
      for (const entry of entries) {
        const { target } = entry;
        if (target === this.mapContainer?.nativeElement) {
          this.isLoading = true;
        } else if (this.matCard && target === this.matCard.nativeElement) {
          const { width, height } = entry.contentRect;
          if (width < height) {
            this.mapContainer.nativeElement.style.width = '100%';
            this.mapContainer.nativeElement.style.height = '';
          } else {
            this.mapContainer.nativeElement.style.height = '100%';
            this.mapContainer.nativeElement.style.width = '';
          }
        }
      }
    });

    const container = this.mapContainer.nativeElement;
    if (container) {
      this.resizeObserver.observe(container);
    }

    if (this.matCard && this.matCard.nativeElement) {
      const matCard = this.matCard.nativeElement;
      this.resizeObserver.observe(matCard);
    }
  }
}
