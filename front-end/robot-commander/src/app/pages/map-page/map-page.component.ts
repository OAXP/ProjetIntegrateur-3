import { Component, OnInit } from '@angular/core';
import { NavigationEnd, Router } from '@angular/router';
import { filter } from 'rxjs';

@Component({
  selector: 'app-map-page',
  templateUrl: './map-page.component.html',
  styleUrls: ['./map-page.component.css']
})
export class MapPageComponent implements OnInit {

  constructor(private router: Router) { }

  ngOnInit(): void {
    setTimeout(() => {
      this.setContainerHeight();
    }, 100);

    window.addEventListener('resize', () => {
      this.setContainerHeight();
    });

    this.router.events.pipe(
      filter(event => event instanceof NavigationEnd)
    ).subscribe(() => {
      setTimeout(() => {
        this.setContainerHeight();
      }, 100);
    });
  }

  private setContainerHeight(): void {
    const toolbar = document.querySelector('.mat-toolbar') as HTMLElement;
    const background = document.querySelector('.background') as HTMLElement;
    const container = document.querySelector('.container') as HTMLElement;
    if (toolbar && background && container) {
      const toolbarStyle = window.getComputedStyle(toolbar);
      const toolbarHeight = parseInt(toolbarStyle.height);
      background.style.height = `calc(100vh - ${toolbarHeight}px)`;

      const backgroundHeight = background.offsetHeight;
      const backgroundWidth = background.offsetWidth;

      if (backgroundWidth < backgroundHeight) {
        container.style.height = 'calc(100vw - 32px)'
        background.style.justifyContent = 'flex-start';
      } else {
        container.style.height = '';
        background.style.justifyContent = 'center';
      }
    }
  }
}
