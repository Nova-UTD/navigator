import LaunchEditorPanel from './launch-editor-panel.svelte';
import type { SubsystemInfo, Node } from '$lib/api';

export type LaunchEditorProps = {
	subsystems: SubsystemInfo[];
	activeNodes: Node[];
	onNodeChanged: (node: Node, selected: boolean) => void;
};

export { LaunchEditorPanel };
