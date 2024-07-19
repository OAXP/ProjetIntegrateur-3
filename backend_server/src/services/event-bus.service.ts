import { Service } from 'typedi';

@Service()
export class EventBus {
    private listeners = new Map<string, Function[]>();

    subscribe(event: string, callback: Function) {
        if (!this.listeners.has(event)) {
            this.listeners.set(event, []);
        }
        this.listeners.get(event)!.push(callback);
    }

    publish(event: string, data?: any) {
        if (this.listeners.has(event)) {
            this.listeners.get(event)!.forEach(callback => callback(data));
        }
    }
}
