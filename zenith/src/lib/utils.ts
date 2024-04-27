import { type ClassValue, clsx } from 'clsx';
import { twMerge } from 'tailwind-merge';
import { cubicOut } from 'svelte/easing';
import type { TransitionConfig } from 'svelte/transition';
import type { Node } from './api';
import { invoke } from '@tauri-apps/api';

/**
 * @param name - The name of the environment variable
 * @param defaultValue - The default value to return if the environment variable is not set
 * @returns - Returns the value of the environment variable or the default value if it is not set
 * @example
 * get_env('HOME') // '/home/user'
 */
export async function get_env(name: string, defaultValue: string = ''): Promise<string> {
	return await invoke('get_env', { name, defaultValue });
}

/**
 * @param node - The node to compare
 * @param otherNode  - The other node to compare
 * @returns  - Returns true if the nodes are equal, false otherwise
 * @example
 * isNodeEqual({ package: 'p1', executable: 'e1' }, { package: 'p1', executable: 'e1' }) // true
 */
export function isNodeEqual(node: Node, otherNode: Node): boolean {
	return node.package === otherNode.package && node.executable === otherNode.executable;
}

/*
 * @param array - The array to chunk
 * @param size - The size of each chunk
 * @returns - Returns a new array with the chunks
 */
export function chunkArray<T>(array: T[], size: number): T[][] {
	return Array.from({ length: Math.ceil(array.length / size) }, (_, i) =>
		array.slice(i * size, i * size + size)
	);
}

/**
 * @param path - The path to get the filename from
 * @returns - Returns the filename from the path
 * @example
 * filenameFromPath('/home/user/file.txt') // 'file.txt'
 */
export function filenameFromPath(path: string): string {
	return path.replace(/^.*[\\/]/, '');
}

/**
 * @returns - Returns a new time-sequential ID
 */
export function seqID(): string {
	return (Date.now() + 1).toString(36);
}

/**
 * @param inputs - The class values to merge
 * @returns - Returns the merged class values with resolved conflicting classes
 * @example
 * cn('text-sm', 'text-lg') // 'text-lg'
 * cn('text-lg', 'text-sm') // 'text-sm'
 */
export function cn(...inputs: ClassValue[]) {
	return twMerge(clsx(inputs));
}

type FlyAndScaleParams = {
	y?: number;
	x?: number;
	start?: number;
	duration?: number;
};

export const flyAndScale = (
	node: Element,
	params: FlyAndScaleParams = { y: -8, x: 0, start: 0.95, duration: 150 }
): TransitionConfig => {
	const style = getComputedStyle(node);
	const transform = style.transform === 'none' ? '' : style.transform;

	const scaleConversion = (valueA: number, scaleA: [number, number], scaleB: [number, number]) => {
		const [minA, maxA] = scaleA;
		const [minB, maxB] = scaleB;

		const percentage = (valueA - minA) / (maxA - minA);
		const valueB = percentage * (maxB - minB) + minB;

		return valueB;
	};

	const styleToString = (style: Record<string, number | string | undefined>): string => {
		return Object.keys(style).reduce((str, key) => {
			if (style[key] === undefined) return str;
			return str + `${key}:${style[key]};`;
		}, '');
	};

	return {
		duration: params.duration ?? 200,
		delay: 0,
		css: (t) => {
			const y = scaleConversion(t, [0, 1], [params.y ?? 5, 0]);
			const x = scaleConversion(t, [0, 1], [params.x ?? 0, 0]);
			const scale = scaleConversion(t, [0, 1], [params.start ?? 0.95, 1]);

			return styleToString({
				transform: `${transform} translate3d(${x}px, ${y}px, 0) scale(${scale})`,
				opacity: t
			});
		},
		easing: cubicOut
	};
};
